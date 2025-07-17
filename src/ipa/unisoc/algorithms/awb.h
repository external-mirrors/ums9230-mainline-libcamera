/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * awb.h - Automatic white balance control based on Unisoc DCAM statistics
 */

#pragma once

#include <libcamera/base/utils.h>

#include "libipa/awb.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Awb : public Algorithm
{
public:
	Awb();
	~Awb() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &info) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     UnisocParams *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const sprd_dcam_statistics *stats,
		     ControlList &metadata) override;

private:
	std::unique_ptr<AwbAlgorithm> awbAlgo_;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
