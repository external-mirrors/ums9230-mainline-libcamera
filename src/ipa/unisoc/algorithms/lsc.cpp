/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Lens shading correction for the Unisoc DCAM pipeline
 */

#include "lsc.h"

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::unisoc::algorithms {

LOG_DEFINE_CATEGORY(UnisocLsc)

Lsc::Lsc()
{
}

int Lsc::init([[maybe_unused]] IPAContext &context, const YamlObject &tuningData)
{
	/* \todo Support multiple configurations for different resolutions. */

	blkSize_ = tuningData["blkSize"].get<uint16_t>(0);
	blkNumX_ = tuningData["blkNumX"].get<uint16_t>(0);
	blkNumY_ = tuningData["blkNumY"].get<uint16_t>(0);
	if (!blkSize_ || !blkNumX_ || !blkNumY_) {
		LOG(UnisocLsc, Error) << "Failed to parse grid configuration";
		return -EINVAL;
	}

	if (blkSize_ / 2 >= SPRD_CAMSYS_LSC_MAX_WEIGHTS) {
		LOG(UnisocLsc, Error)
			<< "Too many weights required! Decrease the block size";
		return -EINVAL;
	}

	grid_ = tuningData["grid"].getList<uint16_t>().value_or(std::vector<uint16_t>{});
	if (grid_.size() != blkNumX_ * blkNumY_ * 4) {
		LOG(UnisocLsc, Error)
			<< "Grid has is missing or has incorrect size "
			<< grid_.size();
		return -EINVAL;
	}

	return 0;
}

int Lsc::configure([[maybe_unused]] IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	/* \todo Ensure that the image size matches the grid configuration. */

	return 0;
}

void Lsc::prepare([[maybe_unused]] IPAContext &context, const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  UnisocParams *params)
{
	if (frame == 0) {
		auto lscCfg = params->dcamBlock<BlockType::Lsc>();

		lscCfg->blk_width = blkSize_;
		lscCfg->blk_num_x = blkNumX_;
		lscCfg->blk_num_y = blkNumY_;

		LOG(UnisocLsc, Debug) << "Configuring " << blkNumX_
				      << "x" << blkNumY_ << " grid";

		/* Use linear interpolation between neighboring blocks */
		for (unsigned int i = 0; i <= (blkSize_ / 2); i++) {
			lscCfg->weights[i][0] = 0;
			lscCfg->weights[i][1] = 1024 - (1024 * i / blkSize_);
			lscCfg->weights[i][2] = 1024 * i / blkSize_;
		}

		params->lscGridSize_ = blkNumX_ * blkNumY_ * 4 * sizeof(uint16_t);
		if (params->lscGridSize_ > params->lscGridData_.size()) {
			LOG(UnisocLsc, Error) << "Grid does not fit into HW buffer!";
			params->lscGridSize_ = 0;
		} else {
			std::copy(grid_.begin(), grid_.end(),
				  reinterpret_cast<uint16_t *>(params->lscGridData_.data()));
		}
	}
}

REGISTER_IPA_ALGORITHM(Lsc, "Lsc")

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
