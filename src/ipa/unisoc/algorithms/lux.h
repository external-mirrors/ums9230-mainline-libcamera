/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * lux.h - Lux estimation based on Unisoc DCAM statistics
 */

#pragma once

#include <sys/types.h>

#include "libipa/lux.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::unisoc::algorithms {

class Lux : public Algorithm
{
public:
	Lux();

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const sprd_dcam_statistics *stats,
		     ControlList &metadata) override;

private:
	ipa::Lux lux_;
};

} /* namespace ipa::unisoc::algorithms */

} /* namespace libcamera */
