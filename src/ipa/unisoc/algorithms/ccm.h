/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * ccm.h - Unisoc ISP color correction matrix
 */

#pragma once

#include <libcamera/base/utils.h>

#include "libcamera/internal/matrix.h"

#include "libipa/interpolator.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Ccm : public Algorithm
{
public:
	Ccm();
	~Ccm() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     UnisocParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const sprd_dcam_statistics *stats,
		     ControlList &metadata) override;

private:
	Interpolator<Matrix<float, 3, 3>> ccm_;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
