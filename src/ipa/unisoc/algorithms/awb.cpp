/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Automatic white balance control based on Unisoc DCAM statistics
 */

#include "awb.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/awb_bayes.h"
#include "libipa/fixedpoint.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

LOG_DEFINE_CATEGORY(UnisocAwb)

/*
 * Minimum proportion of well-exposed pixels in a statistics block for the
 * block to be included in the average.
 */
static constexpr double kMinValidPixelRatio = 0.2;

/*
 * Minimum average which at least one color channel must exceed for the
 * algorithm to operate.
 */
static constexpr double kMeanMinThreshold = 16.0;

static constexpr int32_t kMinColourTemperature = 2500;
static constexpr int32_t kMaxColourTemperature = 10000;
static constexpr int32_t kDefaultColourTemperature = 5000;

/* \todo Deduplicate: this is copied from RkISP1. */
class UnisocAwbStats final : public AwbStats
{
public:
	UnisocAwbStats(const RGB<double> &rgbMeans)
		: rgbMeans_(rgbMeans)
	{
		rg_ = rgbMeans_.r() / rgbMeans_.g();
		bg_ = rgbMeans_.b() / rgbMeans_.g();
	}

	double computeColourError(const RGB<double> &gains) const override
	{
		/*
		 * Compute the sum of the squared colour error (non-greyness) as
		 * it appears in the log likelihood equation.
		 */
		double deltaR = gains.r() * rg_ - 1.0;
		double deltaB = gains.b() * bg_ - 1.0;
		double delta2 = deltaR * deltaR + deltaB * deltaB;

		return delta2;
	}

	RGB<double> rgbMeans() const override
	{
		return rgbMeans_;
	}

private:
	RGB<double> rgbMeans_;
	double rg_;
	double bg_;
};

Awb::Awb()
{
}

int Awb::init(IPAContext &context, const YamlObject &tuningData)
{
	context.ctrlMap[&controls::ColourTemperature] =
		ControlInfo(kMinColourTemperature, kMaxColourTemperature,
			    kDefaultColourTemperature);

	awbAlgo_ = std::make_unique<AwbBayes>();

	int ret = awbAlgo_->init(tuningData);
	if (ret) {
		LOG(UnisocAwb, Error) << "Failed to initialize Bayes AWB algorithm";
		return ret;
	}

	const auto &src = awbAlgo_->controls();
	context.ctrlMap.insert(src.begin(), src.end());

	return 0;
}

int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &info)
{
	context.activeState.awb.temperatureK = kDefaultColourTemperature;
	auto gains = awbAlgo_->gainsFromColourTemperature(kDefaultColourTemperature);
	if (gains)
		context.activeState.awb.gains = *gains;
	else
		context.activeState.awb.gains = RGB<double>{ 1.0 };

	return 0;
}

void Awb::queueRequest([[maybe_unused]] IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       [[maybe_unused]] IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	awbAlgo_->handleControls(controls);
}

void Awb::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, UnisocParams *params)
{
	auto awbCfg = params->dcamBlock<BlockType::Awb>();

	RGB<double> gains = context.activeState.awb.gains;

	awbCfg->gain_r = floatingToFixedPoint<4, 10, uint16_t, double>(gains.r());
	awbCfg->gain_b = floatingToFixedPoint<4, 10, uint16_t, double>(gains.b());
	awbCfg->gain_gr = floatingToFixedPoint<4, 10, uint16_t, double>(gains.g());
	awbCfg->gain_gb = floatingToFixedPoint<4, 10, uint16_t, double>(gains.g());

	frameContext.awb.gains = gains;
	frameContext.awb.temperatureK = context.activeState.awb.temperatureK;
}

void Awb::process(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, const sprd_dcam_statistics *stats,
		  ControlList &metadata)
{
	metadata.set(controls::ColourGains, {
		static_cast<float>(frameContext.awb.gains.r()),
		static_cast<float>(frameContext.awb.gains.b())
	});
	metadata.set(controls::ColourTemperature, frameContext.awb.temperatureK);

	uint32_t rSum = 0, gSum = 0, bSum = 0, counted = 0;

	IPASessionConfiguration &configuration = context.configuration;
	uint32_t pixelsPerBlock = configuration.grid.pixelsPerBlock;

	for (uint32_t i = 0; i < configuration.grid.numBlocks; i++) {
		const sprd_camsys_ae_block_stats *blk = &stats->ae.blk[i];

		uint32_t rCnt = (pixelsPerBlock / 4) - blk->r.low_cnt - blk->r.high_cnt;
		uint32_t gCnt = (pixelsPerBlock / 2) - blk->g.low_cnt - blk->g.high_cnt;
		uint32_t bCnt = (pixelsPerBlock / 4) - blk->b.low_cnt - blk->b.high_cnt;
		uint32_t minCnt = pixelsPerBlock * kMinValidPixelRatio;

		if (rCnt < minCnt || gCnt < minCnt || bCnt < minCnt)
			continue;

		rSum += blk->r.mid_sum / rCnt;
		gSum += blk->g.mid_sum / gCnt;
		bSum += blk->b.mid_sum / bCnt;
		counted++;
	}

	if (counted == 0)
		return;

	RGB<double> means{ {
		static_cast<double>(rSum) / counted,
		static_cast<double>(gSum) / counted,
		static_cast<double>(bSum) / counted
	} };

	if (means.r() < kMeanMinThreshold && means.g() < kMeanMinThreshold &&
	    means.b() < kMeanMinThreshold)
		return;

	UnisocAwbStats awbStats{ means };
	AwbResult awbResult = awbAlgo_->calculateAwb(awbStats, frameContext.lux.lux);

	metadata.set(controls::ColourTemperature, awbResult.colourTemperature);

	IPAActiveState &activeState = context.activeState;
	activeState.awb.temperatureK = awbResult.colourTemperature;
	activeState.awb.gains = awbResult.gains * 0.2 + activeState.awb.gains * 0.8;

	LOG(UnisocAwb, Debug)
		<< "Means: " << means << ", gains: " << activeState.awb.gains
		<< ", temp: " << activeState.awb.temperatureK << "K";
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
