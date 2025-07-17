/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025 Otto Pflüger
 *
 * Pipeline Handler for Unisoc SoCs
 */

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/property_ids.h>
#include <libcamera/stream.h>

#include <libcamera/ipa/unisoc_ipa_interface.h>
#include <libcamera/ipa/unisoc_ipa_proxy.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"


namespace libcamera {

LOG_DEFINE_CATEGORY(Unisoc)

class UnisocISPStream;
class PipelineHandlerUnisoc;

class UnisocDcamDevice {
public:
	UnisocDcamDevice(std::unique_ptr<V4L2Subdevice> sd,
			 std::unique_ptr<V4L2VideoDevice> config,
			 std::unique_ptr<V4L2VideoDevice> lsc,
			 std::unique_ptr<V4L2VideoDevice> fullCap,
			 std::unique_ptr<V4L2VideoDevice> stats)
		: sd_(std::move(sd)), config_(std::move(config)), lsc_(std::move(lsc)),
		  fullCap_(std::move(fullCap)), stats_(std::move(stats)),
		  rawStream_(nullptr), available_(true)
	{
	}

	std::unique_ptr<V4L2Subdevice> sd_;
	std::unique_ptr<V4L2VideoDevice> config_;
	std::unique_ptr<V4L2VideoDevice> lsc_;
	std::unique_ptr<V4L2VideoDevice> fullCap_;
	std::unique_ptr<V4L2VideoDevice> stats_;
	Stream *rawStream_;
	bool available_;
};

class UnisocISPContext {
public:
	UnisocISPContext(std::unique_ptr<V4L2VideoDevice> config,
			 std::unique_ptr<V4L2VideoDevice> input,
			 std::unique_ptr<V4L2VideoDevice> output)
		: config_(std::move(config)), input_(std::move(input)),
		  output_(std::move(output)), stream_(nullptr)
	{
	}

	int configure(StreamConfiguration *config, V4L2DeviceFormat *inputFormat);
	int start();
	void stop();

	std::unique_ptr<V4L2VideoDevice> config_;
	std::unique_ptr<V4L2VideoDevice> input_;
	std::unique_ptr<V4L2VideoDevice> output_;
	UnisocISPStream *stream_;
};

class UnisocISPStream : public Stream
{
public:
	UnisocISPStream()
		: configured_(false), context_(nullptr)
	{
	}

	bool configured_;
	UnisocISPContext *context_;
};

struct UnisocFrameInfo {
	Request *request;

	FrameBuffer *dcamCfgBuffer;
	FrameBuffer *dcamLscBuffer;
	FrameBuffer *ispCfgBuffer;
	FrameBuffer *captureBuffer;
	FrameBuffer *statsBuffer;

	bool statsDone;
};

class UnisocCameraData : public Camera::Private
{
public:
	UnisocCameraData(PipelineHandler *pipe, unsigned int numStreams)
		: Camera::Private(pipe), rawConfigured_(false),
		  streams_(numStreams), dcam_(nullptr)
	{
	}

	PipelineHandlerUnisoc *pipe();
	const PipelineHandlerUnisoc *pipe() const;

	UnisocISPContext *contextForStream(const Stream *stream)
	{
		for (UnisocISPStream &s : streams_) {
			if (stream == &s)
				return s.context_;
		}

		return nullptr;
	}

	int initialize();

	void updateControls(const ControlInfoMap &ipaControls);

	void dcamCfgBufferReady(FrameBuffer *buffer);
	void dcamLscBufferReady(FrameBuffer *buffer);
	void ispCfgBufferReady(FrameBuffer *buffer);
	void captureBufferReady(FrameBuffer *buffer);
	void statsBufferReady(FrameBuffer *buffer);
	void returnCaptureBuffer(FrameBuffer *buffer);
	void processedBufferReady(FrameBuffer *buffer);
	void queuePendingRequests();

	bool rawConfigured_;
	Stream rawStream_;

	std::vector<UnisocISPStream> streams_;

	std::vector<std::unique_ptr<FrameBuffer>> dcamCfgBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> dcamLscBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> ispCfgBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> captureBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statsBuffers_;
	std::queue<FrameBuffer *> availableDcamCfgBuffers_;
	std::queue<FrameBuffer *> availableDcamLscBuffers_;
	std::queue<FrameBuffer *> availableIspCfgBuffers_;
	std::queue<FrameBuffer *> availableCaptureBuffers_;
	std::queue<FrameBuffer *> availableStatsBuffers_;

	std::unique_ptr<ipa::unisoc::IPAProxyUnisoc> ipa_;
	std::vector<IPABuffer> ipaStatBuffers_;
	std::vector<IPABuffer> ipaParamBuffers_;

	std::map<unsigned int, UnisocFrameInfo> frameInfos_;

	std::queue<Request *> pendingRequests_;
	std::map<FrameBuffer *, unsigned int> processingBuffers_;

	UnisocDcamDevice *dcam_;

	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<DelayedControls> delayedCtrls_;

	Size defaultSize_;
	SizeRange processedSizeRange_;

private:
	UnisocFrameInfo *findFrameInfo(unsigned int frame)
	{
		auto it = frameInfos_.find(frame);
		if (it != frameInfos_.end())
			return &it->second;

		return nullptr;
	}

	UnisocFrameInfo *findFrameInfo(Request *request)
	{
		UnisocFrameInfo *info = findFrameInfo(request->sequence());
		ASSERT(info->request == request);
		return info;
	}

	void paramsComputed(unsigned int requestId, unsigned int dcamCfgSize,
			    unsigned int lscDataSize, unsigned int ispCfgSize);
	void statsProcessed(unsigned int requestId, const ControlList &metadata);
	void setSensorControls(const ControlList &sensorControls);

	void tryComplete(UnisocFrameInfo *info);
};

class UnisocCameraConfiguration : public CameraConfiguration
{
public:
	UnisocCameraConfiguration(UnisocCameraData *data)
		: CameraConfiguration(), data_(data)
	{
	}

	Status validate() override;

	V4L2SubdeviceFormat sensorFormat_;

private:
	Status validateRaw(StreamConfiguration *cfg);

	const UnisocCameraData *data_;
};

class PipelineHandlerUnisoc : public PipelineHandler
{
public:
	PipelineHandlerUnisoc(CameraManager *manager)
		: PipelineHandler(manager)
	{
	}

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	V4L2VideoDevice *getTestCapture() const
	{
		return dcams_[0].fullCap_.get();
	}

	V4L2VideoDevice *getTestOutput() const
	{
		return contexts_[0].output_.get();
	}

	Size maxInputSize_;
	std::set<PixelFormat> ispOutFormats_;

private:
	UnisocCameraData *cameraData(Camera *camera)
	{
		return static_cast<UnisocCameraData *>(camera->_d());
	}

	bool openDcam(unsigned int idx);
	bool openContext(unsigned int idx);

	std::shared_ptr<MediaDevice> camsys_;
	std::shared_ptr<MediaDevice> isp_;

	std::vector<UnisocDcamDevice> dcams_;
	std::vector<UnisocISPContext> contexts_;
};

namespace {

constexpr unsigned int kNumInternalBuffers = 4;
constexpr Size kViewfinderSize = { 1280, 960 };
constexpr unsigned int kMaxUpscaling = 4;
constexpr unsigned int kMaxDownscaling = 10;

const std::map<PixelFormat, unsigned int> supportedRawFormats = {
	{ formats::SBGGR10_CSI2P, MEDIA_BUS_FMT_SBGGR10_1X10 },
	{ formats::SGBRG10_CSI2P, MEDIA_BUS_FMT_SGBRG10_1X10 },
	{ formats::SGRBG10_CSI2P, MEDIA_BUS_FMT_SGRBG10_1X10 },
	{ formats::SRGGB10_CSI2P, MEDIA_BUS_FMT_SRGGB10_1X10 },
};

const std::map<unsigned int, PixelFormat> mbusCodeToCaptureFormat = {
	{ MEDIA_BUS_FMT_SBGGR10_1X10, formats::SBGGR10_CSI2P },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, formats::SGBRG10_CSI2P },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, formats::SGRBG10_CSI2P },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, formats::SRGGB10_CSI2P },
};

} /* namespace */

int UnisocISPContext::configure(StreamConfiguration *config, V4L2DeviceFormat *inputFormat)
{
	int ret;

	ret = input_->setFormat(inputFormat);
	if (ret < 0)
		return ret;

	V4L2DeviceFormat outputFormat;
	outputFormat.fourcc = output_->toV4L2PixelFormat(config->pixelFormat);
	outputFormat.size = config->size;
	outputFormat.colorSpace = config->colorSpace;

	ret = output_->setFormat(&outputFormat);
	if (ret < 0)
		return ret;

	return 0;
}

int UnisocISPContext::start()
{
	int ret;

	ret = config_->importBuffers(kNumInternalBuffers);
	if (ret < 0)
		return ret;

	ret = config_->streamOn();
	if (ret < 0)
		return ret;

	ret = input_->importBuffers(kNumInternalBuffers);
	if (ret < 0)
		return ret;

	ret = input_->streamOn();
	if (ret < 0)
		return ret;

	ret = output_->importBuffers(stream_->configuration().bufferCount);
	if (ret < 0)
		return ret;

	ret = output_->streamOn();
	if (ret < 0)
		return ret;

	return 0;
}

void UnisocISPContext::stop()
{
	output_->streamOff();
	output_->releaseBuffers();
	input_->streamOff();
	input_->releaseBuffers();
	config_->streamOff();
	config_->releaseBuffers();
}

PipelineHandlerUnisoc *UnisocCameraData::pipe()
{
	return static_cast<PipelineHandlerUnisoc *>(Camera::Private::pipe());
}

const PipelineHandlerUnisoc *UnisocCameraData::pipe() const
{
	return static_cast<const PipelineHandlerUnisoc *>(Camera::Private::pipe());
}

int UnisocCameraData::initialize()
{
	int ret;

	properties_ = sensor_->properties();

	const CameraSensorProperties::SensorDelays &delays = sensor_->sensorDelays();
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
		{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
	};

	delayedCtrls_ = std::make_unique<DelayedControls>(sensor_->device(), params);

	ipa_ = IPAManager::createIPA<ipa::unisoc::IPAProxyUnisoc>(pipe(), 1, 1);
	if (!ipa_)
		return -ENOENT;

	ipa_->statsProcessed.connect(this, &UnisocCameraData::statsProcessed);
	ipa_->paramsComputed.connect(this, &UnisocCameraData::paramsComputed);
	ipa_->setSensorControls.connect(this, &UnisocCameraData::setSensorControls);

	std::string ipaTuningFile = ipa_->configurationFile(sensor_->model() + ".yaml",
							    "uncalibrated.yaml");

	ipa::unisoc::IPAConfigInfo ipaConfig{};

	ret = sensor_->sensorInfo(&ipaConfig.sensorInfo);
	if (ret)
		return ret;

	ipaConfig.sensorControls = sensor_->controls();

	ControlInfoMap ipaControls;
	ret = ipa_->init({ ipaTuningFile, sensor_->model() }, ipaConfig, &ipaControls);
	if (ret) {
		LOG(Unisoc, Error) << "Failed to initialize the IPA";
		return ret;
	}

	updateControls(ipaControls);

	return 0;
}

void UnisocCameraData::updateControls(const ControlInfoMap &ipaControls)
{
	ControlInfoMap::Map controls;

	for (const auto &c : ipaControls)
		controls.emplace(c.first, c.second);

	controlInfo_ = ControlInfoMap(std::move(controls), controls::controls);
}

void UnisocCameraData::dcamCfgBufferReady(FrameBuffer *buffer)
{
	availableDcamCfgBuffers_.push(buffer);
	queuePendingRequests();
}

void UnisocCameraData::dcamLscBufferReady(FrameBuffer *buffer)
{
	availableDcamLscBuffers_.push(buffer);
	queuePendingRequests();
}

void UnisocCameraData::ispCfgBufferReady(FrameBuffer *buffer)
{
	auto it = processingBuffers_.find(buffer);
	if (it == processingBuffers_.end())
		return;

	if (!--it->second) {
		processingBuffers_.erase(it);
		availableIspCfgBuffers_.push(buffer);
		queuePendingRequests();
	}
}

void UnisocCameraData::captureBufferReady(FrameBuffer *buffer)
{
	PipelineHandlerUnisoc *pipe = UnisocCameraData::pipe();
	Request *request = buffer->request();

	/* Skip ISP processing for raw streams. */
	if (rawConfigured_) {
		pipe->completeBuffer(request, buffer);
		if (!request->hasPendingBuffers())
			pipe->completeRequest(request);

		return;
	}

	/*
	 * If the capture buffer failed, cancel all buffers and complete
	 * the request.
	 */
	if (buffer->metadata().status != FrameMetadata::FrameSuccess) {
		LOG(Unisoc, Debug) << *request << " capture failed";

		for (auto it : request->buffers()) {
			FrameBuffer *b = it.second;
			b->_d()->cancel();
			pipe->completeBuffer(request, b);
		}

		frameInfos_.erase(request->sequence());
		pipe->completeRequest(request);
		return;
	}

	UnisocFrameInfo *info = findFrameInfo(request);
	ASSERT(info);

	LOG(Unisoc, Debug) << *request << " capture done";

	request->_d()->metadata().set(controls::SensorTimestamp,
				      buffer->metadata().timestamp);

	const Request::BufferMap &outBuffers = request->buffers();
	for (auto &[stream, outBuffer] : outBuffers) {
		UnisocISPContext *ctx = contextForStream(stream);
		if (!ctx) {
			LOG(Unisoc, Fatal)
				<< "Stream " << stream << " not found!";
		}

		if (info->ispCfgBuffer)
			ctx->config_->queueBuffer(info->ispCfgBuffer);
		ctx->input_->queueBuffer(buffer);
		ctx->output_->queueBuffer(outBuffer);
	}

	processingBuffers_.emplace(buffer, outBuffers.size());
	if (info->ispCfgBuffer)
		processingBuffers_.emplace(info->ispCfgBuffer, outBuffers.size());
}

void UnisocCameraData::statsBufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();
	UnisocFrameInfo *info = findFrameInfo(request);
	if (!info) {
		/* This can happen when the request is canceled. */
		LOG(Unisoc, Debug)
			<< "Dropping stats buffer " << buffer;
		availableStatsBuffers_.push(buffer);
		queuePendingRequests();
		return;
	}

	ControlList sensorControls = delayedCtrls_->get(buffer->metadata().sequence);

	ipa_->processStats(request->sequence(), buffer->cookie(),
			   sensorControls);
}

void UnisocCameraData::returnCaptureBuffer(FrameBuffer *buffer)
{
	auto it = processingBuffers_.find(buffer);
	if (it == processingBuffers_.end())
		return;

	if (!--it->second) {
		processingBuffers_.erase(it);
		availableCaptureBuffers_.push(buffer);
		queuePendingRequests();
	}
}

void UnisocCameraData::processedBufferReady(FrameBuffer *buffer)
{
	PipelineHandlerUnisoc *pipe = UnisocCameraData::pipe();
	Request *request = buffer->request();
	UnisocFrameInfo *info = findFrameInfo(request);
	ASSERT(info);

	if (pipe->completeBuffer(request, buffer))
		tryComplete(info);
}

void UnisocCameraData::paramsComputed(unsigned int frame,
				      unsigned int dcamCfgSize,
				      unsigned int lscDataSize,
				      unsigned int ispCfgSize)
{
	UnisocFrameInfo *info = findFrameInfo(frame);
	if (!info)
		return;

	if (dcamCfgSize != 0) {
		info->dcamCfgBuffer->_d()->metadata().planes()[0].bytesused = dcamCfgSize;
		dcam_->config_->queueBuffer(info->dcamCfgBuffer);
	} else {
		availableDcamCfgBuffers_.push(info->dcamCfgBuffer);
		queuePendingRequests();
	}

	if (lscDataSize != 0) {
		info->dcamLscBuffer->_d()->metadata().planes()[0].bytesused = lscDataSize;
		dcam_->lsc_->queueBuffer(info->dcamLscBuffer);
	} else {
		availableDcamLscBuffers_.push(info->dcamLscBuffer);
		queuePendingRequests();
	}

	if (ispCfgSize != 0) {
		info->ispCfgBuffer->_d()->metadata().planes()[0].bytesused = ispCfgSize;
	} else {
		availableIspCfgBuffers_.push(info->ispCfgBuffer);
		info->ispCfgBuffer = nullptr;
		queuePendingRequests();
	}

	dcam_->stats_->queueBuffer(info->statsBuffer);
	dcam_->fullCap_->queueBuffer(info->captureBuffer);
}

void UnisocCameraData::statsProcessed(unsigned int frame, const ControlList &metadata)
{
	UnisocFrameInfo *info = findFrameInfo(frame);
	if (!info)
		return;

	info->statsDone = true;
	info->request->_d()->metadata().merge(metadata);

	tryComplete(info);
}

void UnisocCameraData::setSensorControls(const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
}

void UnisocCameraData::tryComplete(UnisocFrameInfo *info)
{
	Request *request = info->request;

	LOG(Unisoc, Debug)
		<< *request
		<< " stats " << (info->statsDone ? "done" : "pending");

	if (!info->statsDone)
		return;

	if (request->hasPendingBuffers())
		return;

	frameInfos_.erase(request->sequence());

	availableStatsBuffers_.push(info->statsBuffer);
	/* The capture buffer should already be returned by now. */

	pipe()->completeRequest(request);

	queuePendingRequests();
}

void UnisocCameraData::queuePendingRequests()
{
	while (!pendingRequests_.empty()) {
		if (availableDcamCfgBuffers_.empty()) {
			LOG(Unisoc, Debug) << "DCAM config buffer underrun";
			break;
		}

		if (availableDcamLscBuffers_.empty()) {
			LOG(Unisoc, Debug) << "DCAM LSC buffer underrun";
			break;
		}

		if (availableIspCfgBuffers_.empty()) {
			LOG(Unisoc, Debug) << "ISP config buffer underrun";
			break;
		}

		if (availableCaptureBuffers_.empty()) {
			LOG(Unisoc, Debug) << "Capture buffer underrun";
			break;
		}

		if (availableStatsBuffers_.empty()) {
			LOG(Unisoc, Debug) << "Stats buffer underrun";
			break;
		}

		Request *request = pendingRequests_.front();
		pendingRequests_.pop();

		UnisocFrameInfo info;
		info.request = request;

		info.dcamCfgBuffer = availableDcamCfgBuffers_.front();
		availableDcamCfgBuffers_.pop();

		info.dcamLscBuffer = availableDcamLscBuffers_.front();
		availableDcamLscBuffers_.pop();

		info.ispCfgBuffer = availableIspCfgBuffers_.front();
		availableIspCfgBuffers_.pop();

		info.captureBuffer = availableCaptureBuffers_.front();
		info.captureBuffer->_d()->setRequest(request);
		availableCaptureBuffers_.pop();

		info.statsBuffer = availableStatsBuffers_.front();
		info.statsBuffer->_d()->setRequest(request);
		availableStatsBuffers_.pop();

		info.statsDone = false;

		LOG(Unisoc, Debug) << *request << " started";

		frameInfos_[request->sequence()] = info;

		ipa_->queueRequest(request->sequence(), request->controls());
		ipa_->computeParams(request->sequence(),
				    info.dcamCfgBuffer->cookie(),
				    info.dcamLscBuffer->cookie(),
				    info.ispCfgBuffer->cookie());
	}
}

CameraConfiguration::Status UnisocCameraConfiguration::validateRaw(StreamConfiguration *cfg)
{
	CameraSensor *sensor = data_->sensor_.get();
	Status status = Valid;

	const auto &rawFmt = supportedRawFormats.find(cfg->pixelFormat);
	if (rawFmt == supportedRawFormats.end())
		return Invalid;

	sensorFormat_ = sensor->getFormat(std::array{ rawFmt->second }, cfg->size);
	if (sensorFormat_.size.isNull())
		return Invalid;

	if (sensorFormat_.size != cfg->size) {
		cfg->size = sensorFormat_.size;
		status = Adjusted;
	}

	V4L2VideoDevice *capture = data_->pipe()->getTestCapture();

	V4L2DeviceFormat format;
	format.fourcc = capture->toV4L2PixelFormat(cfg->pixelFormat);
	format.size = cfg->size;

	int ret = capture->tryFormat(&format);
	if (ret < 0)
		return Invalid;

	cfg->stride = format.planes[0].bpl;
	cfg->frameSize = format.planes[0].size;

	if (cfg->colorSpace != ColorSpace::Raw) {
		cfg->colorSpace = ColorSpace::Raw;
		status = Adjusted;
	}

	return status;
}

CameraConfiguration::Status UnisocCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	if (config_.size() > data_->streams_.size()) {
		config_.resize(data_->streams_.size());
		status = Adjusted;
	}

	for (StreamConfiguration &config : config_) {
		if (PixelFormatInfo::info(config.pixelFormat).colourEncoding ==
		    PixelFormatInfo::ColourEncodingRAW) {
			if (config_.size() != 1) {
				LOG(Unisoc, Error)
					<< "Raw capture not supported with multiple streams";
				return Invalid;
			}

			return validateRaw(&config_[0]);
		}
	}

	V4L2VideoDevice *testOutput = data_->pipe()->getTestOutput();

	Size maxRequestedSize{ 0, 0 };

	for (StreamConfiguration &config : config_) {
		if (!data_->pipe()->ispOutFormats_.count(config.pixelFormat)) {
			config.pixelFormat = formats::NV12;
			status = Adjusted;
		}

		Size size = config.size.expandedTo(data_->processedSizeRange_.min)
				       .alignedUpTo(2, 2)
				       .boundedTo(data_->processedSizeRange_.max);
		if (size != config.size) {
			config.size = size;
			status = Adjusted;
		}

		maxRequestedSize.expandTo(size);

		V4L2DeviceFormat format;
		format.fourcc = testOutput->toV4L2PixelFormat(config.pixelFormat);
		format.size = size;
		format.colorSpace = config.colorSpace;

		int ret = testOutput->tryFormat(&format);
		if (ret < 0)
			return Invalid;

		config.stride = format.planes[0].bpl;
		config.frameSize = format.planes[0].size;

		if (config.colorSpace != format.colorSpace) {
			config.colorSpace = format.colorSpace;
			status = Adjusted;
		}
	}

	std::vector<unsigned int> mbusCodes = utils::map_keys(mbusCodeToCaptureFormat);
	CameraSensor *sensor = data_->sensor_.get();

	sensorFormat_ = sensor->getFormat(mbusCodes, maxRequestedSize,
					  data_->pipe()->maxInputSize_);
	if (sensorFormat_.size.isNull())
		sensorFormat_.size = sensor->resolution();

	return status;
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerUnisoc::generateConfiguration(Camera *camera,
					     Span<const StreamRole> roles)
{
	UnisocCameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<UnisocCameraConfiguration>(data);

	if (roles.empty())
		return config;

	for (const StreamRole &role : roles) {
		PixelFormat pixelFormat = formats::NV12;
		ColorSpace colorSpace = ColorSpace::Raw;
		Size size;

		switch (role) {
		case StreamRole::Raw:
			/* Ignore ISP size limits since raw streams bypass the ISP. */
			size = data->sensor_->resolution();
			break;

		case StreamRole::StillCapture:
			colorSpace = ColorSpace::Sycc;
			size = data->defaultSize_;
			break;

		case StreamRole::Viewfinder:
			colorSpace = ColorSpace::Sycc;
			size = kViewfinderSize.expandedToAspectRatio(data->defaultSize_)
					      .boundedTo(data->defaultSize_);
			break;

		case StreamRole::VideoRecording:
			colorSpace = ColorSpace::Rec709;
			size = kViewfinderSize.expandedToAspectRatio(data->defaultSize_)
					      .boundedTo(data->defaultSize_);
			break;

		default:
			LOG(Unisoc, Error)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		std::map<PixelFormat, std::vector<SizeRange>> formats;

		if (role == StreamRole::Raw) {
			/*
			 * For raw streams, list the sizes and formats
			 * supported by the sensor since the ISP is bypassed.
			 * Select the raw format with the highest number of
			 * bits per pixel by default.
			 */
			unsigned int maxBitsPerPixel = 0;

			for (const auto &[rawFmt, mbusCode] : supportedRawFormats) {
				const auto sizes = data->sensor_->sizes(mbusCode);
				if (sizes.empty())
					continue;

				std::vector<SizeRange> sizeRanges;
				std::transform(sizes.begin(), sizes.end(),
					       std::back_inserter(sizeRanges),
					       [](const Size &s) {
						       return SizeRange(s);
					       });

				formats[rawFmt] = sizeRanges;

				const PixelFormatInfo &info = PixelFormatInfo::info(rawFmt);
				if (info.bitsPerPixel > maxBitsPerPixel) {
					maxBitsPerPixel = info.bitsPerPixel;
					pixelFormat = rawFmt;
				}
			}

			if (!maxBitsPerPixel) {
				LOG(Unisoc, Error)
					<< "Failed to find a supported raw format";
				return nullptr;
			}
		} else {
			size.expandTo(data->processedSizeRange_.min);
			size.boundTo(data->processedSizeRange_.max);

			for (const PixelFormat &pixFmt : ispOutFormats_)
				formats[pixFmt] = { data->processedSizeRange_ };
		}

		StreamConfiguration cfg{ StreamFormats{ formats } };
		cfg.bufferCount = 4;
		cfg.size = size;
		cfg.pixelFormat = pixelFormat;
		cfg.colorSpace = colorSpace;

		config->addConfiguration(cfg);
	}

	if (config->validate() == CameraConfiguration::Invalid) {
		LOG(Unisoc, Debug) << "Cannot build a valid configuration";
		return nullptr;
	}

	return config;
}

int PipelineHandlerUnisoc::configure(Camera *camera, CameraConfiguration *c)
{
	UnisocCameraConfiguration *config =
		static_cast<UnisocCameraConfiguration *>(c);
	UnisocCameraData *data = cameraData(camera);
	int ret;

	/*
	 * To make switching configurations possible, first release all
	 * resources used by the previous configuration.
	 */
	releaseDevice(camera);

	V4L2SubdeviceFormat subdevFormat = config->sensorFormat_;

	LOG(Unisoc, Debug) << "Configuring sensor format: " << subdevFormat;

	ret = data->sensor_->setFormat(&subdevFormat);
	if (ret < 0)
		return ret;

	LOG(Unisoc, Debug) << "Configured sensor format: " << subdevFormat;

	for (UnisocDcamDevice &dcam : dcams_) {
		if (dcam.available_) {
			data->dcam_ = &dcam;
			dcam.available_ = false;
			break;
		}
	}

	if (!data->dcam_) {
		LOG(Unisoc, Error) << "All DCAM instances are busy";
		return -EBUSY;
	}

	const MediaEntity *dcamEntity = data->dcam_->sd_->entity();
	const MediaPad *sensorSource = data->sensor_->entity()->getPadByIndex(0);
	const MediaPad *dcamSink = dcamEntity->getPadByIndex(0);

	/*
	 * Find an unused CSI controller that can be used in the pipeline
	 * between the sensor and the selected DCAM instance.
	 */
	MediaLink *selectedDcamLink = nullptr;
	MediaLink *selectedCsiLink = nullptr;
	MediaEntity *csiEntity = nullptr;
	for (MediaLink *csiLink : sensorSource->links()) {
		/* If a link was already found, just disable the other ones. */
		if (selectedCsiLink) {
			csiLink->setEnabled(false);
			continue;
		}

		csiEntity = csiLink->sink()->entity();

		for (MediaLink *dcamLink : csiEntity->getPadByIndex(1)->links()) {
			if (dcamLink->sink() == dcamSink) {
				selectedDcamLink = dcamLink;
			} else if (dcamLink->flags() & MEDIA_LNK_FL_ENABLED) {
				LOG(Unisoc, Debug)
					<< csiEntity->name()
					<< " is already in use with "
					<< dcamLink->sink()->entity()->name();
				selectedDcamLink = nullptr;
				break;
			}
		}

		/*
		 * Use this CSI controller for the camera if a link from it
		 * to the selected DCAM instance is available.
		 */
		if (selectedDcamLink)
			selectedCsiLink = csiLink;
		else
			csiLink->setEnabled(false);
	}

	if (!selectedCsiLink) {
		LOG(Unisoc, Error) << "Could not link sensor to DCAM";
		return -ENOLINK;
	}

	LOG(Unisoc, Debug)
		<< "Connecting " << data->sensor_->entity()->name()
		<< " to " << dcamEntity->name()
		<< " via " << csiEntity->name();

	ret = selectedCsiLink->setEnabled(true);
	if (ret < 0)
		return ret;

	ret = selectedDcamLink->setEnabled(true);
	if (ret < 0)
		return ret;

	std::unique_ptr<V4L2Subdevice> csi =
		std::make_unique<V4L2Subdevice>(csiEntity);
	if (csi->open() < 0)
		return ret;

	ret = csi->setFormat(0, &subdevFormat);
	if (ret < 0)
		return ret;

	ret = data->dcam_->sd_->setFormat(0, &subdevFormat);
	if (ret < 0)
		return ret;

	V4L2VideoDevice *capture = data->dcam_->fullCap_.get();

	const auto &fmtIter = mbusCodeToCaptureFormat.find(subdevFormat.code);
	if (fmtIter == mbusCodeToCaptureFormat.end()) {
		LOG(Unisoc, Error) << "DCAM capture format not supported";
		return -EINVAL;
	}

	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = capture->toV4L2PixelFormat(fmtIter->second);
	captureFormat.size = subdevFormat.size;
	ret = capture->setFormat(&captureFormat);
	if (ret < 0)
		return -EINVAL;

	LOG(Unisoc, Debug) << "DCAM capture format: " << captureFormat;

	/* Disable all streams by default and then selectively enable them. */
	data->rawConfigured_ = false;
	for (UnisocISPStream &stream : data->streams_) {
		if (stream.context_) {
			stream.context_->stream_ = nullptr;
			stream.context_ = nullptr;
		}
	}

	for (unsigned int i = 0; i < config->size(); i++) {
		StreamConfiguration &cfg = config->at(i);

		if (PixelFormatInfo::info(cfg.pixelFormat).colourEncoding ==
		    PixelFormatInfo::ColourEncodingRAW) {
			data->rawConfigured_ = true;
			cfg.setStream(&data->rawStream_);
			maxOutputSize = cfg.size;
			break;
		}

		UnisocISPStream &stream = data->streams_[i];
		for (UnisocISPContext &ctx : contexts_) {
			if (!ctx.stream_) {
				stream.context_ = &ctx;
				ctx.stream_ = &stream;
				break;
			}
		}

		if (!stream.context_) {
			LOG(Unisoc, Error)
				<< "Not enough ISP contexts available";
			return -EBUSY;
		}

		stream.context_->configure(&cfg, &captureFormat);
		cfg.setStream(&stream);
	}

	ipa::unisoc::IPAConfigInfo ipaConfig;

	ret = data->sensor_->sensorInfo(&ipaConfig.sensorInfo);
	if (ret)
		return ret;

	ipaConfig.sensorControls = data->sensor_->controls();

	ControlInfoMap ipaControls;
	ret = data->ipa_->configure(ipaConfig, &ipaControls);
	if (ret) {
		LOG(Unisoc, Error) << "Failed to configure the IPA";
		return ret;
	}

	data->updateControls(ipaControls);

	return 0;
}

int PipelineHandlerUnisoc::exportFrameBuffers(Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	UnisocCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	if (stream == &data->rawStream_) {
		return data->dcam_->fullCap_->exportBuffers(count, buffers);
	} else {
		UnisocISPContext *ctx = data->contextForStream(stream);
		if (!ctx) {
			LOG(Unisoc, Error) << "Stream " << stream << " not found!";
			return -ENODEV;
		}

		return ctx->output_->exportBuffers(count, buffers);
	}
}

int PipelineHandlerUnisoc::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	UnisocCameraData *data = cameraData(camera);
	int ret;

	data->dcam_->sd_->frameStart.connect(data->delayedCtrls_.get(),
					     &DelayedControls::applyControls);

	V4L2VideoDevice *dcamCfg = data->dcam_->config_.get();
	V4L2VideoDevice *dcamLsc = data->dcam_->lsc_.get();
	V4L2VideoDevice *capture = data->dcam_->fullCap_.get();
	V4L2VideoDevice *stats = data->dcam_->stats_.get();

	dcamCfg->bufferReady.connect(data, &UnisocCameraData::dcamCfgBufferReady);
	dcamLsc->bufferReady.connect(data, &UnisocCameraData::dcamLscBufferReady);
	capture->bufferReady.connect(data, &UnisocCameraData::captureBufferReady);
	stats->bufferReady.connect(data, &UnisocCameraData::statsBufferReady);

	for (UnisocISPStream &stream : data->streams_) {
		if (!stream.context_)
			continue;

		V4L2VideoDevice *ispCfg = stream.context_->config_.get();
		V4L2VideoDevice *input = stream.context_->input_.get();
		V4L2VideoDevice *output = stream.context_->output_.get();
		ispCfg->bufferReady.connect(data, &UnisocCameraData::ispCfgBufferReady);
		input->bufferReady.connect(data, &UnisocCameraData::returnCaptureBuffer);
		output->bufferReady.connect(data, &UnisocCameraData::processedBufferReady);
	}

	if (data->rawConfigured_) {
		Stream *stream = &data->rawStream_;
		data->dcam_->rawStream_ = stream;

		ret = capture->importBuffers(stream->configuration().bufferCount);
		if (ret < 0) {
			stopDevice(camera);
			return ret;
		}

		ret = capture->streamOn();
		if (ret < 0) {
			stopDevice(camera);
			return ret;
		}

		return 0;
	}

	ret = capture->allocateBuffers(kNumInternalBuffers, &data->captureBuffers_);
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	for (std::unique_ptr<FrameBuffer> &buffer : data->captureBuffers_)
		data->availableCaptureBuffers_.push(buffer.get());

	unsigned int ipaBufferId = 1;

	auto pushBuffers = [&](const std::vector<std::unique_ptr<FrameBuffer>> &buffers,
			       std::queue<FrameBuffer *> &queue,
			       std::vector<IPABuffer> &ipaBuffers) {
		for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
			Span<const FrameBuffer::Plane> planes = buffer->planes();

			buffer->setCookie(ipaBufferId++);
			ipaBuffers.emplace_back(buffer->cookie(),
						std::vector<FrameBuffer::Plane>{ planes.begin(),
										 planes.end() });
			queue.push(buffer.get());
		}
	};

	ret = stats->allocateBuffers(kNumInternalBuffers, &data->statsBuffers_);
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	pushBuffers(data->statsBuffers_, data->availableStatsBuffers_,
		    data->ipaStatBuffers_);

	ret = dcamCfg->allocateBuffers(kNumInternalBuffers, &data->dcamCfgBuffers_);
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	pushBuffers(data->dcamCfgBuffers_, data->availableDcamCfgBuffers_,
		    data->ipaParamBuffers_);

	ret = dcamLsc->allocateBuffers(kNumInternalBuffers, &data->dcamLscBuffers_);
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	pushBuffers(data->dcamLscBuffers_, data->availableDcamLscBuffers_,
		    data->ipaParamBuffers_);

	ret = contexts_[0].config_->exportBuffers(kNumInternalBuffers,
						  &data->ispCfgBuffers_);
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	pushBuffers(data->ispCfgBuffers_, data->availableIspCfgBuffers_,
		    data->ipaParamBuffers_);

	data->ipa_->mapBuffers(data->ipaStatBuffers_, true);
	data->ipa_->mapBuffers(data->ipaParamBuffers_, false);
	ret = data->ipa_->start();
	if (ret) {
		LOG(Unisoc, Error) << "Failed to start IPA";
		stopDevice(camera);
		return ret;
	}

	for (UnisocISPStream &stream : data->streams_) {
		if (!stream.context_)
			continue;

		ret = stream.context_->start();
		if (ret < 0) {
			stopDevice(camera);
			return ret;
		}
	}

	ret = dcamCfg->streamOn();
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	ret = dcamLsc->streamOn();
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	ret = capture->streamOn();
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	ret = stats->streamOn();
	if (ret < 0) {
		stopDevice(camera);
		return ret;
	}

	ret = data->dcam_->sd_->setFrameStartEnabled(true);
	if (ret)
		LOG(Unisoc, Error) << "Failed to enable frame start events";

	return 0;
}

void PipelineHandlerUnisoc::stopDevice(Camera *camera)
{
	UnisocCameraData *data = cameraData(camera);

	data->dcam_->sd_->setFrameStartEnabled(false);

	while (!data->pendingRequests_.empty()) {
		cancelRequest(data->pendingRequests_.front());
		data->pendingRequests_.pop();
	}

	data->dcam_->stats_->streamOff();
	data->dcam_->fullCap_->streamOff();
	data->dcam_->lsc_->streamOff();
	data->dcam_->config_->streamOff();

	for (UnisocISPStream &stream : data->streams_) {
		if (stream.context_)
			stream.context_->stop();
	}

	data->ipa_->stop();
	data->ipa_->unmapBuffers(data->ipaParamBuffers_);
	data->ipa_->unmapBuffers(data->ipaStatBuffers_);
	data->ipaParamBuffers_.clear();
	data->ipaStatBuffers_.clear();

	data->frameInfos_.clear();

	while (!data->availableStatsBuffers_.empty())
		data->availableStatsBuffers_.pop();
	while (!data->availableCaptureBuffers_.empty())
		data->availableCaptureBuffers_.pop();
	while (!data->availableIspCfgBuffers_.empty())
		data->availableIspCfgBuffers_.pop();
	while (!data->availableDcamLscBuffers_.empty())
		data->availableDcamLscBuffers_.pop();
	while (!data->availableDcamCfgBuffers_.empty())
		data->availableDcamCfgBuffers_.pop();

	data->statsBuffers_.clear();
	data->captureBuffers_.clear();
	data->ispCfgBuffers_.clear();
	data->dcamLscBuffers_.clear();
	data->dcamCfgBuffers_.clear();

	data->dcam_->stats_->releaseBuffers();
	data->dcam_->fullCap_->releaseBuffers();
	data->dcam_->lsc_->releaseBuffers();
	data->dcam_->config_->releaseBuffers();

	for (UnisocISPStream &stream : data->streams_) {
		if (!stream.context_)
			continue;

		V4L2VideoDevice *ispCfg = stream.context_->config_.get();
		V4L2VideoDevice *input = stream.context_->input_.get();
		V4L2VideoDevice *output = stream.context_->output_.get();
		ispCfg->bufferReady.disconnect(data, &UnisocCameraData::ispCfgBufferReady);
		input->bufferReady.disconnect(data, &UnisocCameraData::returnCaptureBuffer);
		output->bufferReady.disconnect(data, &UnisocCameraData::processedBufferReady);
	}

	data->dcam_->stats_->bufferReady.disconnect(data, &UnisocCameraData::statsBufferReady);
	data->dcam_->fullCap_->bufferReady.disconnect(data, &UnisocCameraData::captureBufferReady);
	data->dcam_->lsc_->bufferReady.disconnect(data, &UnisocCameraData::dcamLscBufferReady);
	data->dcam_->config_->bufferReady.disconnect(data, &UnisocCameraData::dcamCfgBufferReady);
	data->dcam_->sd_->frameStart.disconnect(data->delayedCtrls_.get(),
						&DelayedControls::applyControls);
}

void PipelineHandlerUnisoc::releaseDevice(Camera *camera)
{
	UnisocCameraData *data = cameraData(camera);

	for (UnisocISPStream &stream : data->streams_) {
		if (stream.context_) {
			stream.context_->stream_ = nullptr;
			stream.context_ = nullptr;
		}
	}

	if (data->dcam_) {
		data->dcam_->available_ = true;
		data->dcam_ = nullptr;
	}

	/* Disable active links to free pipeline resources for other cameras. */
	const MediaPad *sensorSource = data->sensor_->entity()->getPadByIndex(0);
	for (MediaLink *csiLink : sensorSource->links()) {
		if (!(csiLink->flags() & MEDIA_LNK_FL_ENABLED))
			continue;

		csiLink->setEnabled(false);

		MediaEntity *csiEntity = csiLink->sink()->entity();

		for (MediaLink *dcamLink : csiEntity->getPadByIndex(1)->links())
			dcamLink->setEnabled(false);
	}
}

int PipelineHandlerUnisoc::queueRequestDevice(Camera *camera, Request *request)
{
	UnisocCameraData *data = cameraData(camera);

	if (data->rawConfigured_) {
		FrameBuffer *buffer = request->findBuffer(&data->rawStream_);
		ASSERT(buffer);
		data->dcam_->fullCap_->queueBuffer(buffer);
	} else {
		data->pendingRequests_.push(request);
		data->queuePendingRequests();
	}

	return 0;
}

bool PipelineHandlerUnisoc::openDcam(unsigned int idx)
{
	std::string name = "sprd_dcam" + std::to_string(idx);

	std::unique_ptr<V4L2Subdevice> sd =
		V4L2Subdevice::fromEntityName(camsys_.get(), name);
	if (!sd) {
		LOG(Unisoc, Debug) << "Cannot find " << name;
		return false;
	}

	if (sd->open() < 0)
		return false;

	std::string configName = name + "_config";

	std::unique_ptr<V4L2VideoDevice> config =
		V4L2VideoDevice::fromEntityName(camsys_.get(), configName);
	if (!config) {
		LOG(Unisoc, Debug) << "Cannot find " << configName;
		return false;
	}

	if (config->open() < 0)
		return false;

	std::string lscName = name + "_lsc";

	std::unique_ptr<V4L2VideoDevice> lsc =
		V4L2VideoDevice::fromEntityName(camsys_.get(), lscName);
	if (!lsc) {
		LOG(Unisoc, Debug) << "Cannot find " << lscName;
		return false;
	}

	if (lsc->open() < 0)
		return false;

	std::string fullName = name + "_full";

	std::unique_ptr<V4L2VideoDevice> fullCap =
		V4L2VideoDevice::fromEntityName(camsys_.get(), fullName);
	if (!fullCap) {
		LOG(Unisoc, Debug) << "Cannot find " << fullName;
		return false;
	}

	if (fullCap->open() < 0)
		return false;

	std::string statsName = name + "_stats";

	std::unique_ptr<V4L2VideoDevice> stats =
		V4L2VideoDevice::fromEntityName(camsys_.get(), statsName);
	if (!stats) {
		LOG(Unisoc, Debug) << "Cannot find " << statsName;
		return false;
	}

	if (stats->open() < 0)
		return false;

	dcams_.emplace_back(std::move(sd), std::move(config), std::move(lsc),
			    std::move(fullCap), std::move(stats));

	return true;
}

bool PipelineHandlerUnisoc::openContext(unsigned int idx)
{
	std::string prefix = "sprd_isp_" + std::to_string(idx);
	std::string cfgName = prefix + "_config";
	std::string inName = prefix + "_input";
	std::string outName = prefix + "_output";

	std::unique_ptr<V4L2VideoDevice> config =
		V4L2VideoDevice::fromEntityName(isp_.get(), cfgName);
	if (!config) {
		LOG(Unisoc, Debug) << "Cannot find " << cfgName;
		return false;
	}

	if (config->open() < 0)
		return false;

	std::unique_ptr<V4L2VideoDevice> input =
		V4L2VideoDevice::fromEntityName(isp_.get(), inName);
	if (!input) {
		LOG(Unisoc, Debug) << "Cannot find " << inName;
		return false;
	}

	if (input->open() < 0)
		return false;

	std::unique_ptr<V4L2VideoDevice> output =
		V4L2VideoDevice::fromEntityName(isp_.get(), outName);
	if (!output) {
		LOG(Unisoc, Debug) << "Cannot find " << outName;
		return false;
	}

	if (output->open() < 0)
		return false;

	contexts_.emplace_back(std::move(config), std::move(input),
			       std::move(output));

	return true;
}

bool PipelineHandlerUnisoc::match(DeviceEnumerator *enumerator)
{
	DeviceMatch camsys("sprd-camsys");
	camsys.add("sprd_csi0");
	camsys.add("sprd_dcam0_config");
	camsys.add("sprd_dcam0_lsc");
	camsys.add("sprd_dcam0_full");
	camsys.add("sprd_dcam0_stats");
	camsys_ = acquireMediaDevice(enumerator, camsys);

	if (!camsys_) {
		LOG(Unisoc, Debug) << "Unable to acquire CAMSYS instance";
		return false;
	}

	DeviceMatch isp("sprd-isp");
	isp.add("sprd_isp_0_config");
	isp.add("sprd_isp_0_input");
	isp.add("sprd_isp_0_output");
	isp_ = acquireMediaDevice(enumerator, isp);

	if (!isp_) {
		LOG(Unisoc, Debug) << "Unable to acquire ISP instance";
		return false;
	}

	for (unsigned i = 0; ; i++) {
		if (!openDcam(i))
			break;
	}

	if (dcams_.empty()) {
		LOG(Unisoc, Error) << "Failed to open a DCAM instance";
		return false;
	}

	for (unsigned i = 0; ; i++) {
		if (!openContext(i))
			break;
	}

	if (contexts_.empty()) {
		LOG(Unisoc, Error) << "Failed to open an ISP context";
		return false;
	}

	/* Enumerate the sizes and formats supported by the ISP. */
	Size minOutputSize{ 65535, 65535 };
	Size maxOutputSize{ 0, 0 };
	for (const auto &[format, sizes] : contexts_[0].output_->formats()) {
		const PixelFormat pixFmt = format.toPixelFormat();

		ispOutFormats_.insert(pixFmt);

		for (const SizeRange &range : sizes) {
			minOutputSize.boundTo(range.min);
			maxOutputSize.expandTo(range.max);
		}
	}

	maxInputSize_ = { 0, 0 };
	for (const auto &[format, sizes] : contexts_[0].input_->formats()) {
		for (const SizeRange &range : sizes)
			maxInputSize_.expandTo(range.max);
	}

	/*
	 * Register all cameras connected to the first CSI controller.
	 * This should find every available camera since the hardware
	 * supports multiplexing.
	 */
	std::unique_ptr<V4L2Subdevice> subdev =
		V4L2Subdevice::fromEntityName(camsys_.get(), "sprd_csi0");
	const MediaPad *csiSink = subdev->entity()->getPadByIndex(0);

	bool registered = false;
	for (MediaLink *link : csiSink->links()) {
		MediaEntity *sensor = link->source()->entity();
		if (!sensor)
			continue;

		if (sensor->function() != MEDIA_ENT_F_CAM_SENSOR) {
			LOG(Unisoc, Debug) << "Ignoring non-sensor entity "
					   << sensor->name()
					   << " connected to CSI0";
			continue;
		}

		std::unique_ptr<UnisocCameraData> data =
			std::make_unique<UnisocCameraData>(this, contexts_.size());

		data->sensor_ = CameraSensorFactoryBase::create(sensor);
		if (!data->sensor_)
			continue;

		/* Find the intersection of the sensor's and the ISP's size range. */
		Size minResolution{ 65535, 65535 };
		Size maxResolution{ 0, 0 };
		for (unsigned int code : data->sensor_->mbusCodes()) {
			for (const Size &size : data->sensor_->sizes(code)) {
				if (size.width > maxOutputSize.width ||
				    size.height > maxOutputSize.height)
					continue;

				minResolution.boundTo(size);
				maxResolution.expandTo(size);
			}
		}

		if (maxResolution.isNull()) {
			LOG(Unisoc, Warning) << "Sensor "
					   << sensor->name()
					   << " has no supported resolutions";
			continue;
		}

		data->defaultSize_ = maxResolution;
		data->processedSizeRange_ = {
			(minResolution / kMaxDownscaling).expandTo(minOutputSize),
			(maxResolution * kMaxUpscaling).boundTo(maxOutputSize)
		};

		if (data->initialize() < 0)
			continue;

		std::set<Stream *> streams{ &data->rawStream_ };
		std::transform(data->streams_.begin(), data->streams_.end(),
			       std::inserter(streams, streams.end()),
			       [](Stream &stream) { return &stream; });

		const std::string &id = data->sensor_->id();
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), id, streams);
		registerCamera(std::move(camera));
		registered = true;
	}

	return registered;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUnisoc, "unisoc")

} /* namespace libcamera */
