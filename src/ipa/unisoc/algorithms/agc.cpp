/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Automatic exposure and gain control based on Unisoc DCAM statistics
 */

#include "agc.h"

#include <chrono>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::unisoc::algorithms {

LOG_DEFINE_CATEGORY(UnisocAgc)

Agc::Agc()
	: AgcMeanLuminance()
{
}

int Agc::init(IPAContext &context, const YamlObject &tuningData)
{
	int ret = parseTuningData(tuningData);
	if (ret)
		return ret;

	context.ctrlMap[&controls::AeEnable] = ControlInfo(false, true, true);
	context.ctrlMap.merge(controls());

	return 0;
}

int Agc::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &info)
{
	IPASessionConfiguration &configuration = context.configuration;
	IPAActiveState &activeState = context.activeState;

	activeState.agc.autoEnabled = true;
	activeState.agc.automatic.sensorGain = configuration.agc.minAnalogueGain;
	activeState.agc.automatic.exposure = configuration.agc.defaultExposure;
	activeState.agc.manual.sensorGain = configuration.agc.minAnalogueGain;
	activeState.agc.manual.exposure = configuration.agc.defaultExposure;

	/* \todo Recalculate when FrameDurationLimits is changed */
	setLimits(configuration.agc.minExposureTime,
		  configuration.agc.maxExposureTime,
		  configuration.agc.minAnalogueGain,
		  configuration.agc.maxAnalogueGain,
		  {});

	resetFrameCount();

	return 0;
}

void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	IPAActiveState &activeState = context.activeState;

	const auto &constraintMode = controls.get(controls::AeConstraintMode);
	if (constraintMode)
		activeState.agc.constraintMode = *constraintMode;

	const auto &exposureMode = controls.get(controls::AeExposureMode);
	if (exposureMode)
		activeState.agc.exposureMode = *exposureMode;

	const auto &aeEnable = controls.get(controls::AeEnable);
	if (aeEnable)
		activeState.agc.autoEnabled = *aeEnable;

	if (activeState.agc.autoEnabled)
		return;

	const auto &exposure = controls.get(controls::ExposureTime);
	if (exposure)
		activeState.agc.manual.exposure = *exposure * 1.0us /
				context.configuration.sensor.lineDuration;

	const auto &analogueGain = controls.get(controls::AnalogueGain);
	if (analogueGain)
		activeState.agc.manual.sensorGain = *analogueGain;
}

double Agc::estimateLuminance(const double gain) const
{
	double rAvg = rHist_.interQuantileMean(0, 1) * gain;
	double gAvg = gHist_.interQuantileMean(0, 1) * gain;
	double bAvg = bHist_.interQuantileMean(0, 1) * gain;
	double yAvg = rec601LuminanceFromRGB({{ rAvg, gAvg, bAvg }});

	return yAvg / 256;
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const sprd_dcam_statistics *stats,
		  ControlList &metadata)
{
	IPASessionConfiguration &configuration = context.configuration;
	IPAActiveState &activeState = context.activeState;

	uint32_t exposure = frameContext.agc.exposure;
	utils::Duration exposureTime = exposure * configuration.sensor.lineDuration;
	double analogueGain = frameContext.agc.sensorGain;
	utils::Duration effectiveExposureValue = exposureTime * analogueGain;

	metadata.set(controls::AnalogueGain, analogueGain);
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());

	uint32_t r[256] = {}, g[256] = {}, b[256] = {}, y[256] = {};

	/*
	 * Generate a histogram for the entire image from the per-block pixel
	 * sums.
	 *
	 * \todo Make the metering mode configurable and apply weights here.
	 */
	for (uint32_t i = 0; i < configuration.grid.numBlocks; i++) {
		const sprd_camsys_ae_block_stats *blk = &stats->ae.blk[i];

		uint32_t rSum = blk->r.low_sum + blk->r.mid_sum + blk->r.high_sum;
		uint32_t gSum = blk->g.low_sum + blk->g.mid_sum + blk->g.high_sum;
		uint32_t bSum = blk->b.low_sum + blk->b.mid_sum + blk->b.high_sum;
		uint32_t rAvg = rSum / (configuration.grid.pixelsPerBlock / 4);
		uint32_t gAvg = gSum / (configuration.grid.pixelsPerBlock / 2);
		uint32_t bAvg = bSum / (configuration.grid.pixelsPerBlock / 4);

		RGB<double> rgbAvg{{ static_cast<double>(rAvg),
				     static_cast<double>(gAvg),
				     static_cast<double>(bAvg) }};
		uint32_t yAvg = rec601LuminanceFromRGB(rgbAvg);

		/* FIXME: are these always 10-bit? */
		r[rAvg >> 2]++;
		g[gAvg >> 2]++;
		b[bAvg >> 2]++;
		y[yAvg >> 2]++;
	}

	rHist_ = Histogram(Span<uint32_t>(r, 256));
	gHist_ = Histogram(Span<uint32_t>(g, 256));
	bHist_ = Histogram(Span<uint32_t>(b, 256));
	yHist_ = Histogram(Span<uint32_t>(y, 256));

	utils::Duration newExposureTime;
	double aGain, qGain, dGain;
	std::tie(newExposureTime, aGain, qGain, dGain) =
		calculateNewEv(activeState.agc.constraintMode,
			       activeState.agc.exposureMode, yHist_,
			       effectiveExposureValue);

	LOG(UnisocAgc, Debug)
		<< "Split exposure time, analogue gain and digital gain are "
		<< newExposureTime << ", " << aGain << " and " << dGain;

	activeState.agc.automatic.exposure = newExposureTime / context.configuration.sensor.lineDuration;
	activeState.agc.automatic.sensorGain = aGain;
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
