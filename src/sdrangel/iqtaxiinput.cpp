#include "iqtaxiinput.h"

#include <algorithm>
#include <QDebug>
#include <QMutexLocker>

#include "SWGDeviceState.h"

#include "device/deviceapi.h"
#include "dsp/dspcommands.h"
#include "util/simpleserializer.h"

MESSAGE_CLASS_DEFINITION(IqtaxiInput::MsgConfigureIqtaxi, Message)
MESSAGE_CLASS_DEFINITION(IqtaxiInput::MsgStartStop, Message)
MESSAGE_CLASS_DEFINITION(IqtaxiInput::MsgReportError, Message)
MESSAGE_CLASS_DEFINITION(IqtaxiInput::MsgReportRfBand, Message)

IqtaxiInput::IqtaxiInput(DeviceAPI *deviceAPI) :
    _deviceAPI(deviceAPI),
    _deviceDescription("IQTAXI Input"),
    _running(false)
{
    const QString hardwareId = deviceAPI ? deviceAPI->getHardwareId() : QString();
    // Discovery serial is an 8-char board string and must not be used as a model name.
    _settings.device_model = iqtaxiModelFromHardwareId(hardwareId.toStdString());

    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(_settings.device_model))
    {
        _deviceDescription = QString::fromLatin1(caps->display_name) + " Input";
        _settings.sample_rate_hz = caps->default_sample_rate_hz;
        _settings.rx_gain = iqtaxiClampGain(_settings.device_model, _settings.rx_gain);
    }

    if (!m_sampleFifo.setSize(1 << 19))
    {
        qWarning() << "IqtaxiInput: sample FIFO allocate failed";
    }
    m_sampleFifo.setLabel(_deviceDescription);
    _deviceAPI->setNbSourceStreams(1);
}

IqtaxiInput::~IqtaxiInput()
{
    if (_running)
    {
        stop();
    }
}

void IqtaxiInput::destroy()
{
    delete this;
}

void IqtaxiInput::init()
{
    applySettings(_settings, true);
}

bool IqtaxiInput::start()
{
    QMutexLocker locker(&_mutex);

    if (_running)
    {
        return true;
    }

    const bool started = _backend.start(
        _settings,
        [this](const int16_t *samples, size_t count, uint64_t ts) { onSamples(samples, count, ts); },
        [this](const std::string &err) { onError(err); });

    _running = started;
    if (started)
    {
        std::string label = _backend.board_label();
        if (label.empty() && _settings.device_model.find("E100") != std::string::npos)
        {
            const std::string band = _backend.rf_band();
            label = band.empty() ? "E100" : ("E100-" + band);
        }
        if (label.empty())
        {
            label = _settings.device_model;
        }
        _deviceDescription = QString("IQTAXI %1 Input").arg(QString::fromStdString(label));
        m_sampleFifo.setLabel(_deviceDescription);
        qDebug() << "IqtaxiInput started:" << _deviceDescription;
        if (m_guiMessageQueue)
        {
            m_guiMessageQueue->push(MsgReportRfBand::create(QString::fromStdString(label)));
        }
    }
    return started;
}

void IqtaxiInput::stop()
{
    QMutexLocker locker(&_mutex);
    _backend.stop();
    _running = false;
}

QByteArray IqtaxiInput::serialize() const
{
    SimpleSerializer s(1);
    s.writeString(1, QString::fromStdString(_settings.device_addr));
    s.writeU32(2, _settings.sample_rate_hz);
    s.writeU64(3, _settings.center_freq_hz);
    s.writeU32(4, _settings.rx_gain);
    s.writeU32(5, _settings.channel);
    s.writeString(6, QString::fromStdString(_settings.device_model));
    return s.final();
}

bool IqtaxiInput::deserialize(const QByteArray &data)
{
    SimpleDeserializer d(data);
    if (!d.isValid())
    {
        return false;
    }

    QString addr;
    d.readString(1, &addr, "192.168.1.10");
    _settings.device_addr = addr.toStdString();
    d.readU32(2, &_settings.sample_rate_hz, 15360000u);
    quint64 centerFreq = static_cast<quint64>(_settings.center_freq_hz);
    d.readU64(3, &centerFreq, 1000000000ull);
    _settings.center_freq_hz = static_cast<uint64_t>(centerFreq);
    d.readU32(4, &_settings.rx_gain, 30u);
    d.readU32(5, &_settings.channel, 1u);

    QString model;
    d.readString(6, &model, QString::fromStdString(_settings.device_model));
    if (!model.isEmpty())
    {
        _settings.device_model = iqtaxiModelFromHardwareId(model.toStdString());
    }

    // Keep model locked to the SamplingDevice that created this instance.
    if (_deviceAPI)
    {
        _settings.device_model = iqtaxiModelFromHardwareId(
            _deviceAPI->getHardwareId().toStdString());
    }

    _settings.sample_rate_hz = iqtaxiNearestSampleRate(_settings.device_model, _settings.sample_rate_hz);
    _settings.rx_gain = iqtaxiClampGain(_settings.device_model, _settings.rx_gain);

    return _settings.is_valid();
}

const QString &IqtaxiInput::getDeviceDescription() const
{
    return _deviceDescription;
}

int IqtaxiInput::getSampleRate() const
{
    return static_cast<int>(_settings.sample_rate_hz);
}

void IqtaxiInput::setSampleRate(int sampleRate)
{
    IqtaxiSettings next = _settings;
    next.sample_rate_hz = sampleRate > 0 ? static_cast<uint32_t>(sampleRate) : _settings.sample_rate_hz;
    (void)applySettings(next, false);
}

quint64 IqtaxiInput::getCenterFrequency() const
{
    return _settings.center_freq_hz;
}

void IqtaxiInput::setCenterFrequency(qint64 centerFrequency)
{
    IqtaxiSettings next = _settings;
    if (centerFrequency > 0)
    {
        const uint64_t maxHz = iqtaxiMaxCenterFreqHz(_settings.device_model);
        next.center_freq_hz = std::min(static_cast<uint64_t>(centerFrequency), maxHz);
        (void)applySettings(next, false);
    }
}

bool IqtaxiInput::handleMessage(const Message &message)
{
    if (MsgConfigureIqtaxi::match(message))
    {
        const auto &cfg = static_cast<const MsgConfigureIqtaxi &>(message);
        return applySettings(cfg.getSettings(), cfg.getForce());
    }
    if (MsgStartStop::match(message))
    {
        const auto &cmd = static_cast<const MsgStartStop &>(message);
        if (cmd.getStartStop())
        {
            if (!_settings.is_valid())
            {
                if (m_guiMessageQueue)
                {
                    m_guiMessageQueue->push(MsgReportError::create("invalid settings"));
                    m_guiMessageQueue->push(MsgStartStop::create(false));
                }
                qWarning() << "IqtaxiInput start failed: invalid settings";
                return true;
            }

            if (_deviceAPI->initDeviceEngine())
            {
                if (_deviceAPI->startDeviceEngine())
                {
                    auto *notif = new DSPSignalNotification(
                        static_cast<int>(_settings.sample_rate_hz),
                        static_cast<qint64>(_settings.center_freq_hz));
                    _deviceAPI->getDeviceEngineInputMessageQueue()->push(notif);
                }
                else
                {
                    if (m_guiMessageQueue)
                    {
                        m_guiMessageQueue->push(MsgReportError::create("device engine start failed"));
                        m_guiMessageQueue->push(MsgStartStop::create(false));
                    }
                    qWarning() << "IqtaxiInput start failed: startDeviceEngine failed";
                }
            }
            else
            {
                if (m_guiMessageQueue)
                {
                    m_guiMessageQueue->push(MsgReportError::create("device engine init failed"));
                    m_guiMessageQueue->push(MsgStartStop::create(false));
                }
                qWarning() << "IqtaxiInput start failed: initDeviceEngine failed";
            }
            return true;
        }
        _deviceAPI->stopDeviceEngine();
        return true;
    }
    return false;
}

int IqtaxiInput::webapiRunGet(
        SWGSDRangel::SWGDeviceState &response,
        QString &errorMessage)
{
    (void)errorMessage;
    _deviceAPI->getDeviceEngineStateStr(*response.getState());
    return 200;
}

int IqtaxiInput::webapiRun(
        bool run,
        SWGSDRangel::SWGDeviceState &response,
        QString &errorMessage)
{
    (void)errorMessage;
    qDebug() << "IqtaxiInput::webapiRun:" << (run ? "start" : "stop");
    _deviceAPI->getDeviceEngineStateStr(*response.getState());

    // Workspace "Start all devices" uses this path (ChannelWebAPIUtils::run/stop).
    m_inputMessageQueue.push(MsgStartStop::create(run));

    // Update device GUI ButtonSwitch checked state (same as FileInput/USRP).
    if (m_guiMessageQueue)
    {
        m_guiMessageQueue->push(MsgStartStop::create(run));
    }

    return 200;
}

bool IqtaxiInput::applySettings(const IqtaxiSettings &settings, bool force)
{
    QMutexLocker locker(&_mutex);
    if (!settings.is_valid())
    {
        return false;
    }

    if (!force &&
        settings.device_model == _settings.device_model &&
        settings.device_addr == _settings.device_addr &&
        settings.sample_rate_hz == _settings.sample_rate_hz &&
        settings.center_freq_hz == _settings.center_freq_hz &&
        settings.rx_gain == _settings.rx_gain &&
        settings.channel == _settings.channel)
    {
        return true;
    }

    _settings = settings;
    auto *notif = new DSPSignalNotification(
        static_cast<int>(_settings.sample_rate_hz),
        static_cast<qint64>(_settings.center_freq_hz));
    _deviceAPI->getDeviceEngineInputMessageQueue()->push(notif);
    return _backend.apply_settings(_settings);
}

void IqtaxiInput::onSamples(const int16_t *samples, size_t count, uint64_t timestamp)
{
    (void)timestamp;
    if (!samples || count == 0u)
    {
        return;
    }

    SampleVector vec;
    vec.resize(count);
    for (size_t i = 0; i < count; ++i)
    {
        // Match USRP Decimators<...,24,16,...>::decimate1: CS16 -> internal 24-bit.
        // Negate Q: IQTAXI IQ phase is opposite to SDRangel's expected convention (mirrored spectrum).
        const qint32 iSample = static_cast<qint32>(samples[i * 2]);
        const qint32 qSample = static_cast<qint32>(samples[i * 2 + 1]);
#ifdef SDR_RX_SAMPLE_24BIT
        vec[i].m_real = static_cast<FixReal>(iSample << 8);
        vec[i].m_imag = static_cast<FixReal>(qSample << 8);
#else
        vec[i].m_real = static_cast<FixReal>(iSample);
        vec[i].m_imag = static_cast<FixReal>(qSample);
#endif
    }

    m_sampleFifo.write(vec.begin(), vec.end());
}

void IqtaxiInput::onError(const std::string &error)
{
    const QString qerr = QString::fromStdString(error);
    qWarning() << "IqtaxiInput error:" << qerr;
    if (m_guiMessageQueue)
    {
        m_guiMessageQueue->push(MsgReportError::create(qerr));
    }
}
