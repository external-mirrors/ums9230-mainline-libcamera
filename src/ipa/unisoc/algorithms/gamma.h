/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * gamma.h - Unisoc ISP gamma table configuration
 */

#pragma once

#include <libcamera/base/utils.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Gamma : public Algorithm
{
public:
	Gamma();
	~Gamma() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     UnisocParams *params) override;

private:
	std::vector<uint16_t> curveR_;
	std::vector<uint16_t> curveG_;
	std::vector<uint16_t> curveB_;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
