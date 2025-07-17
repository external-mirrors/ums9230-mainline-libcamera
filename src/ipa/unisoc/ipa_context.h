/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * ipa_context.h - Unisoc IPA Context
 */

#pragma once

#include <libcamera/base/utils.h>
#include <libcamera/controls.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::unisoc {

struct IPASessionConfiguration {
	struct {
		utils::Duration minExposureTime;
		utils::Duration maxExposureTime;
		uint32_t defaultExposure;
		double minAnalogueGain;
		double maxAnalogueGain;
	} agc;

	struct {
		uint32_t left;
		uint32_t top;
		uint32_t width;
		uint32_t height;
		uint32_t numBlocks;
		uint32_t blockWidth;
		uint32_t blockHeight;
		uint32_t pixelsPerBlock;
	} grid;

	struct {
		utils::Duration lineDuration;
		uint32_t bitsPerPixel;
		uint16_t blackLevel;
	} sensor;
};

struct IPAActiveState {
	struct {
		struct {
			uint32_t exposure;
			double sensorGain;
		} automatic;
		struct {
			uint32_t exposure;
			double sensorGain;
		} manual;
		bool autoEnabled;
		uint32_t constraintMode;
		uint32_t exposureMode;
	} agc;

	struct {
		RGB<double> gains;
		uint32_t temperatureK;
	} awb;

	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double sensorGain;
	} agc;

	struct {
		RGB<double> gains;
		uint32_t temperatureK;
	} awb;

	struct {
		Matrix<float, 3, 3> ccm;
	} ccm;

	struct {
		double lux;
	} lux;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;
};

} /* namespace ipa::unisoc */

} /* namespace libcamera*/
