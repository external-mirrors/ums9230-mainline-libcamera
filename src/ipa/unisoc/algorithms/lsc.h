/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * lsc.h - Lens shading correction for Unisoc DCAM pipeline
 */

#pragma once

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Lsc : public Algorithm
{
public:
	Lsc();
	~Lsc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     UnisocParams *params) override;

private:
	uint16_t blkSize_;
	uint16_t blkNumX_;
	uint16_t blkNumY_;

	uint16_t weights_[SPRD_CAMSYS_LSC_MAX_WEIGHTS * 3];
	std::vector<uint16_t> grid_;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
