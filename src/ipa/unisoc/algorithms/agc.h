/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * agc.h - Automatic exposure and gain control based on Unisoc DCAM statistics
 */

#pragma once

#include <libcamera/base/utils.h>

#include "libipa/agc_mean_luminance.h"
#include "libipa/histogram.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Agc : public Algorithm, public AgcMeanLuminance
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &info) override;
	void queueRequest(IPAContext &context, const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const sprd_dcam_statistics *stats,
		     ControlList &metadata) override;

private:
	double estimateLuminance(const double gain) const override;

	Histogram rHist_;
	Histogram gHist_;
	Histogram bHist_;
	Histogram yHist_;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
