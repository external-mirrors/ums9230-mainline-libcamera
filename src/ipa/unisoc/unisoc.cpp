/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * unisoc.cpp - Unisoc ISP image processing algorithms
 */

#include <map>
#include <string.h>
#include <vector>

#include <linux/media/sprd/camsys-config.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/unisoc_ipa_interface.h>

#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAUnisoc)

using namespace std::literals::chrono_literals;

namespace ipa::unisoc {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPAUnisoc : public IPAUnisocInterface, public Module
{
public:
	IPAUnisoc();

	int init(const IPASettings &settings, const IPAConfigInfo &ipaConfig,
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;
	int configure(const IPAConfigInfo &ipaConfig,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers, bool readOnly) override;
	void unmapBuffers(const std::vector<IPABuffer> &buffers) override;
	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void computeParams(unsigned int frame, uint32_t dcamBufferId,
			   uint32_t dcamLscBufferId, uint32_t ispBufferId) override;
	void processStats(unsigned int frame, unsigned int bufferId,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateSessionConfiguration(const IPACameraSensorInfo &info,
					const ControlInfoMap &sensorControls);
	void updateControls(const IPACameraSensorInfo &sensorInfo,
			    const ControlInfoMap &sensorControls,
			    ControlInfoMap *ipaControls);
	void setControls();

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap sensorControls_;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

} /* namespace */

IPAUnisoc::IPAUnisoc()
	: context_(kMaxFrameContexts)
{
}

std::string IPAUnisoc::logPrefix() const
{
	return "Unisoc";
}

int IPAUnisoc::init(const IPASettings &settings, const IPAConfigInfo &ipaConfig,
		     ControlInfoMap *ipaControls)
{
	camHelper_ = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(IPAUnisoc, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPAUnisoc, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	if (!data->contains("algorithms")) {
		LOG(IPAUnisoc, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	updateControls(ipaConfig.sensorInfo, ipaConfig.sensorControls, ipaControls);

	return 0;
}

void IPAUnisoc::setControls()
{
	IPAActiveState &activeState = context_.activeState;
	uint32_t exposure;
	uint32_t gain;

	if (activeState.agc.autoEnabled) {
		exposure = activeState.agc.automatic.exposure;
		gain = camHelper_->gainCode(activeState.agc.automatic.sensorGain);
	} else {
		exposure = activeState.agc.manual.exposure;
		gain = camHelper_->gainCode(activeState.agc.manual.sensorGain);
	}

	ControlList ctrls(sensorControls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain));

	setSensorControls.emit(ctrls);
}

int IPAUnisoc::start()
{
	return 0;
}

void IPAUnisoc::stop()
{
	context_.frameContexts.clear();
}

void IPAUnisoc::updateSessionConfiguration(const IPACameraSensorInfo &info,
					   const ControlInfoMap &sensorControls)
{
	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>();
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>();
	int32_t defExposure = v4l2Exposure.def().get<int32_t>();

	const ControlInfo &v4l2Gain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	int32_t minGain = v4l2Gain.min().get<int32_t>();
	int32_t maxGain = v4l2Gain.max().get<int32_t>();

	/*
	 * When the AGC computes the new exposure values for a frame, it needs
	 * to know the limits for exposure time and analogue gain.
	 * As it depends on the sensor, update it with the controls.
	 *
	 * \todo take VBLANK into account for maximum exposure time
	 */
	IPASessionConfiguration &configuration = context_.configuration;
	configuration.sensor.lineDuration = info.minLineLength * 1.0s / info.pixelRate;
	configuration.agc.minExposureTime = minExposure * context_.configuration.sensor.lineDuration;
	configuration.agc.maxExposureTime = maxExposure * context_.configuration.sensor.lineDuration;
	configuration.agc.defaultExposure = defExposure;
	configuration.agc.minAnalogueGain = camHelper_->gain(minGain);
	configuration.agc.maxAnalogueGain = camHelper_->gain(maxGain);

	configuration.sensor.blackLevel = camHelper_->blackLevel().value_or(4096);

	/* 
	 * Use the maximum possible size for the AE/AWB statistics grid.
	 *
	 * \todo Make this configurable.
	 */
	configuration.grid.width = SPRD_CAMSYS_AE_MAX_BLOCKS_X;
	configuration.grid.height = SPRD_CAMSYS_AE_MAX_BLOCKS_Y;

	configuration.grid.left =
		(info.outputSize.width % configuration.grid.width) / 2;
	configuration.grid.top =
		(info.outputSize.height % configuration.grid.height) / 2;

	configuration.grid.blockWidth =
		info.outputSize.width / configuration.grid.width;
	configuration.grid.blockHeight =
		info.outputSize.height / configuration.grid.height;

	/* Store multiplied values for convenience. */
	configuration.grid.numBlocks = configuration.grid.width *
				       configuration.grid.height;
	configuration.grid.pixelsPerBlock = configuration.grid.blockWidth *
					    configuration.grid.blockHeight;
}

void IPAUnisoc::updateControls(const IPACameraSensorInfo &sensorInfo,
				const ControlInfoMap &sensorControls,
				ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map ctrlMap;

	/*
	 * Compute the frame duration limits.
	 *
	 * The frame length is computed assuming a fixed line length combined
	 * with the vertical frame sizes.
	 */
	const ControlInfo &v4l2HBlank = sensorControls.find(V4L2_CID_HBLANK)->second;
	uint32_t hblank = v4l2HBlank.def().get<int32_t>();
	uint32_t lineLength = sensorInfo.outputSize.width + hblank;

	const ControlInfo &v4l2VBlank = sensorControls.find(V4L2_CID_VBLANK)->second;
	std::array<uint32_t, 3> frameHeights{
		v4l2VBlank.min().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.max().get<int32_t>() + sensorInfo.outputSize.height,
		v4l2VBlank.def().get<int32_t>() + sensorInfo.outputSize.height,
	};

	std::array<int64_t, 3> frameDurations;
	for (unsigned int i = 0; i < frameHeights.size(); ++i) {
		uint64_t frameSize = lineLength * frameHeights[i];
		frameDurations[i] = frameSize / (sensorInfo.pixelRate / 1000000U);
	}

	ctrlMap[&controls::FrameDurationLimits] = ControlInfo(frameDurations[0],
							      frameDurations[1],
							      frameDurations[2]);

	/*
	 * Compute exposure time limits from the V4L2_CID_EXPOSURE control
	 * limits and the line duration.
	 */
	double lineDuration = sensorInfo.minLineLength / sensorInfo.pixelRate;

	const ControlInfo &v4l2Exposure = sensorControls.find(V4L2_CID_EXPOSURE)->second;
	int32_t minExposure = v4l2Exposure.min().get<int32_t>() * lineDuration;
	int32_t maxExposure = v4l2Exposure.max().get<int32_t>() * lineDuration;
	int32_t defExposure = v4l2Exposure.def().get<int32_t>() * lineDuration;
	ctrlMap[&controls::ExposureTime] = ControlInfo(minExposure, maxExposure, defExposure);

	/* Compute the analogue gain limits. */
	const ControlInfo &v4l2Gain = sensorControls.find(V4L2_CID_ANALOGUE_GAIN)->second;
	float minGain = camHelper_->gain(v4l2Gain.min().get<int32_t>());
	float maxGain = camHelper_->gain(v4l2Gain.max().get<int32_t>());
	float defGain = camHelper_->gain(v4l2Gain.def().get<int32_t>());
	ctrlMap[&controls::AnalogueGain] = ControlInfo(minGain, maxGain, defGain);

	/*
	 * Merge in any controls that we support either statically or from the
	 * algorithms.
	 */
	ctrlMap.merge(context_.ctrlMap);

	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);
}

int IPAUnisoc::configure(const IPAConfigInfo &ipaConfig, ControlInfoMap *ipaControls)
{
	sensorControls_ = ipaConfig.sensorControls;

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	const IPACameraSensorInfo &info = ipaConfig.sensorInfo;

	updateSessionConfiguration(info, ipaConfig.sensorControls);
	updateControls(info, ipaConfig.sensorControls, ipaControls);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		int ret = algo->configure(context_, info);
		if (ret)
			return ret;
	}

	return 0;
}

void IPAUnisoc::mapBuffers(const std::vector<IPABuffer> &buffers, bool readOnly)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(
			buffer.id,
			MappedFrameBuffer(
				&fb,
				readOnly ? MappedFrameBuffer::MapFlag::Read
					 : MappedFrameBuffer::MapFlag::ReadWrite));
	}
}

void IPAUnisoc::unmapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto it = buffers_.find(buffer.id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(buffer.id);
	}
}

void IPAUnisoc::queueRequest(const uint32_t frame, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(frame);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->queueRequest(context_, frame, frameContext, controls);
	}
}

void IPAUnisoc::computeParams(unsigned int frame, uint32_t dcamBufferId,
			      uint32_t lscBufferId, uint32_t ispBufferId)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	UnisocParams params(buffers_.at(dcamBufferId).planes()[0],
			    buffers_.at(ispBufferId).planes()[0],
			    buffers_.at(lscBufferId).planes()[0]);

	/*
	 * Set up the AE statistics configuration, which is shared between
	 * the AGC and AWB algorithms.
	 */
	if (frame == 0) {
		auto aeCfg = params.dcamBlock<BlockType::Ae>();

		aeCfg->offset_x = context_.configuration.grid.left;
		aeCfg->offset_y = context_.configuration.grid.top;
		aeCfg->blk_num_x = context_.configuration.grid.width;
		aeCfg->blk_num_y = context_.configuration.grid.height;
		aeCfg->blk_width = context_.configuration.grid.blockWidth;
		aeCfg->blk_height = context_.configuration.grid.blockHeight;

		/* \todo Make these configurable. */
		aeCfg->r_low = 32;
		aeCfg->g_low = 32;
		aeCfg->b_low = 32;
		aeCfg->r_high = 768;
		aeCfg->g_high = 768;
		aeCfg->b_high = 768;
	}

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->prepare(context_, frame, frameContext, &params);
	}

	paramsComputed.emit(frame, params.dcamCfg_.bytesused(),
			    params.lscGridSize_, params.ispCfg_.bytesused());
}

void IPAUnisoc::processStats(unsigned int frame, unsigned int bufferId,
			     const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);
	const sprd_dcam_statistics *stats = nullptr;

	stats = reinterpret_cast<sprd_dcam_statistics *>(
		buffers_.at(bufferId).planes()[0].data());

	frameContext.agc.exposure =
		sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	frameContext.agc.sensorGain =
		camHelper_->gain(sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>());

	ControlList metadata(controls::controls);

	for (auto const &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->process(context_, frame, frameContext, stats, metadata);
	}

	setControls();

	statsProcessed.emit(frame, metadata);
}

} /* namespace ipa::unisoc */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"unisoc",
	"unisoc",
};

IPAInterface *ipaCreate()
{
	return new ipa::unisoc::IPAUnisoc();
}

} /* extern "C" */

} /* namespace libcamera */
