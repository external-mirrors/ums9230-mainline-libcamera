/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Automatic focus based on Unisoc DCAM statistics
 */

#include "af.h"

#include <libcamera/control_ids.h>

namespace libcamera {

namespace ipa::unisoc::algorithms {

LOG_DEFINE_CATEGORY(UnisocAf)

static constexpr int kNumStartupFrames = 30;

static constexpr int kMaxLensPos = 1023;

static constexpr int kCoarseStep = 31;
static constexpr int kFineStep = 4;

static constexpr double kFocusLossThreshold = 0.8;

Af::Af()
	: scanning_(false), bestContrast_(0), focus_(0)
{
}

int Af::configure(IPAContext &context, [[maybe_unused]] const IPACameraSensorInfo &info)
{
	IPASessionConfiguration &configuration = context.configuration;

	/*
	 * Use a fixed size for the AF grid that covers the entire image.
	 *
	 * \todo This could be adjusted based on the region of interest.
	 */
	configuration.afGrid.width = 12;
	configuration.afGrid.height = 8;

	configuration.afGrid.left =
		(info.outputSize.width % configuration.afGrid.width) / 2;
	configuration.afGrid.top =
		(info.outputSize.height % configuration.afGrid.height) / 2;

	configuration.afGrid.blockWidth =
		info.outputSize.width / configuration.afGrid.width;
	configuration.afGrid.blockHeight =
		info.outputSize.height / configuration.afGrid.height;

	/* Store multiplied values for convenience. */
	configuration.afGrid.numBlocks = configuration.afGrid.width *
					 configuration.afGrid.height;
	configuration.afGrid.pixelsPerBlock = configuration.afGrid.blockWidth *
					      configuration.afGrid.blockHeight;

	computeWeights(configuration);

	/* Set initial focus value */
	context.activeState.af.focus = focus_;

	frameCount_ = 0;

	return 0;
}

void Af::computeWeights(const IPASessionConfiguration &configuration)
{
	weights_.clear();
	weights_.resize(configuration.afGrid.numBlocks);

	/* \todo Handle the AfWindows control. */
	Rectangle rect(configuration.afGrid.width / 4,
		       configuration.afGrid.height / 4,
		       configuration.afGrid.width / 2,
		       configuration.afGrid.height / 2);
	addWindow(rect, configuration.afGrid.width, 1.0);
}

void Af::addWindow(Rectangle &rect, uint32_t stride, double weight)
{
	weight /= rect.width * rect.height;

	uint32_t i = rect.y * stride + rect.x;

	for (uint32_t y = 0; y < rect.height; y++) {
		for (uint32_t x = 0; x < rect.width; x++)
			weights_[i + x] += weight;

		i += stride;
	}
}

void Af::queueRequest([[maybe_unused]] IPAContext &context,
		      [[maybe_unused]] const uint32_t frame,
		      [[maybe_unused]] IPAFrameContext &frameContext,
		      const ControlList &controls)
{
	const auto &trigger = controls.get(controls::AfTrigger);
	if (trigger) {
		if (*trigger == controls::AfTriggerStart)
			restart();
		else
			scanning_ = false;
	}
}

void Af::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 UnisocParams *params)
{
	if (frameCount_ == 0) {
		auto afCfg = params->dcamBlock<BlockType::Af>();

		afCfg->offset_x = context.configuration.afGrid.left;
		afCfg->offset_y = context.configuration.afGrid.top;
		afCfg->blk_num_x = context.configuration.afGrid.width;
		afCfg->blk_num_y = context.configuration.afGrid.height;
		afCfg->blk_width = context.configuration.afGrid.blockWidth;
		afCfg->blk_height = context.configuration.afGrid.blockHeight;
	}
}

void Af::process(IPAContext &context,
		 [[maybe_unused]] const uint32_t frame,
		 [[maybe_unused]] IPAFrameContext &frameContext,
		 const sprd_dcam_statistics *stats,
		 ControlList &metadata)
{
	IPASessionConfiguration &configuration = context.configuration;

	contrast_ = 0.0;

	for (uint32_t i = 0; i < configuration.afGrid.numBlocks; i++) {
		const sprd_camsys_af_block_stats *blk = &stats->af.blk[i];

		contrast_ += weights_[i] * blk->contrast_1;
	}

	metadata.set(controls::LensPosition, focus_);
	metadata.set(controls::FocusFoM, contrast_);

	/*
	 * Record the maximum contrast. This is used both for scanning and for
	 * detecting loss of focus.
	 */
	if (contrast_ >= bestContrast_) {
		bestContrast_ = contrast_;
		bestFocus_ = focus_;
	}

	if (scanning_) {
		if (scan()) {
			/*
			 * We have just passed the maximum. If the step size is
			 * not small enough yet, refine the search.
			 */
			if (std::abs(step_) > kFineStep) {
				/* Move back to within one step of the maximum. */
				focus_ = bestFocus_ + step_;
				/* Reverse the direction and slow down. */
				step_ = -(step_ / 2);
				retries_ = 1;
				bestContrast_ = 0;
			} else {
				/* Move to the observed maximum. */
				focus_ = bestFocus_;
				scanning_ = false;
				LOG(UnisocAf, Debug) << "Focus done: " << focus_;
			}
		}
	} else if (contrast_ < bestContrast_ * kFocusLossThreshold ||
		   frameCount_ == kNumStartupFrames) {
		restart();
	}

	context.activeState.af.focus = focus_;
	frameCount_++;
}

void Af::restart()
{
	LOG(UnisocAf, Debug) << "Starting AF algorithm";

	bestContrast_ = 0;
	step_ = kCoarseStep;
	retries_ = 2;
	scanning_ = true;
}

bool Af::scan()
{
	LOG(UnisocAf, Debug)
		<< "Scan step " << step_ << ", current: "
		<< focus_ << " -> " << contrast_ << ", best: "
		<< bestFocus_ << " -> " << bestContrast_;

	/*
	 * Check if the contrast has decreased significantly compared to the
	 * observed maximum.
	 */
	if (contrast_ < (bestContrast_ * 0.9))
		return true;

	/*
	 * If the end of the focus range is reached, reverse the direction
	 * and search again, or stop if the search fails repeatedly.
	 */
	if ((step_ < 0) ? (focus_ <= 0) : (focus_ >= kMaxLensPos)) {
		if (--retries_ <= 0)
			return true;

		step_ = -step_;
	}

	/* Shift the focus by one step and wait for the next measurement. */
	focus_ += step_;

	return false;
}

REGISTER_IPA_ALGORITHM(Af, "Af")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
