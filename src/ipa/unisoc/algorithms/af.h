/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * af.h - Automatic focus based on Unisoc DCAM statistics
 */

#pragma once

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Af : public Algorithm
{
public:
	Af();
	~Af() = default;

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
	void computeWeights(const IPASessionConfiguration &configuration);
	void addWindow(Rectangle &rect, uint32_t stride, double weight);

	void restart();
	bool scan();

	std::vector<double> weights_;

	uint64_t frameCount_;

	bool scanning_;
	double contrast_;
	double bestContrast_;
	int focus_;
	int bestFocus_;
	int step_;
	int retries_;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
