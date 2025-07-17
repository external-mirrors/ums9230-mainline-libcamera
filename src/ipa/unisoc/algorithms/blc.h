/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * blc.h - Black level correction for Unisoc DCAM pipeline
 */

#pragma once

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Blc : public Algorithm
{
public:
	Blc();
	~Blc() = default;

	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     UnisocParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const sprd_dcam_statistics *stats,
		     ControlList &metadata) override;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
