/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Lux estimation based on Unisoc DCAM statistics
 */

#include "lux.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"
#include "libipa/histogram.h"
#include "libipa/lux.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

Lux::Lux()
{
}

int Lux::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	return lux_.parseTuningData(tuningData);
}

void Lux::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const sprd_dcam_statistics *stats,
		  ControlList &metadata)
{
	IPASessionConfiguration &configuration = context.configuration;

	uint32_t exposure = frameContext.agc.exposure;
	utils::Duration exposureTime = exposure * configuration.sensor.lineDuration;
	double analogueGain = frameContext.agc.sensorGain;

	/*
	 * Generate a histogram for the entire image from the per-block pixel
	 * sums.
	 *
	 * \todo Deduplicate from the AGC algorithm.
	 */

	uint32_t yBins[256] = {};

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
		yBins[std::clamp<uint32_t>(yAvg >> 2, 0, 255)]++;
	}

	Histogram yHist(Span<uint32_t>(yBins, 256));

	double lux = lux_.estimateLux(exposureTime, analogueGain, 1.0, yHist);
	frameContext.lux.lux = lux;
	metadata.set(controls::Lux, lux);
}

REGISTER_IPA_ALGORITHM(Lux, "Lux")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
