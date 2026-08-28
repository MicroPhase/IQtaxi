#include "iqtaxigui.h"

#include <algorithm>
#include <QFont>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPixmap>
#include <QSizePolicy>
#include <QVariant>
#include <QVBoxLayout>

#include "device/deviceapi.h"
#include "device/deviceuiset.h"
#include "gui/colormapper.h"
#include "include/sdr/api/DeviceProfile.hpp"
#include "include/sdr/api/UdpDiscover.hpp"

namespace
{
constexpr int kFreqDebounceMs = 250;
} // namespace

IqtaxiGui::IqtaxiGui(DeviceUISet *deviceUISet, QWidget *parent) :
    DeviceGUI(parent),
    m_sampleSource((deviceUISet && deviceUISet->m_deviceAPI) ? deviceUISet->m_deviceAPI->getSampleSource() : nullptr),
    m_rootWidget(new QWidget(getContents())),
    m_descLabel(nullptr),
    m_ipEdit(nullptr),
    m_scanButton(nullptr),
    m_freqDial(nullptr),
    m_rateCombo(nullptr),
    m_gainSpin(nullptr),
    m_startStopButton(nullptr),
    m_applyButton(nullptr),
    m_pendingFreqKhz(0),
    m_lastEngineState(-1),
    m_doApplySettings(true),
    m_updatingUi(false)
{
    m_deviceUISet = deviceUISet;
    setAttribute(Qt::WA_DeleteOnClose, true);
    setDeviceType(DeviceGUI::DeviceRx);

    if (deviceUISet && deviceUISet->m_deviceAPI)
    {
        const QString hardwareId = deviceUISet->m_deviceAPI->getHardwareId();
        m_settings.device_model = iqtaxiModelFromHardwareId(hardwareId.toStdString());
    }

    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(m_settings.device_model))
    {
        setTitle(QString::fromLatin1(caps->display_name));
        m_settings.sample_rate_hz = caps->default_sample_rate_hz;
        m_settings.rx_gain = iqtaxiClampGain(m_settings.device_model, m_settings.rx_gain);
    }
    else
    {
        setTitle("IQTAXI");
    }

    m_freqDebounceTimer.setSingleShot(true);
    connect(&m_freqDebounceTimer, &QTimer::timeout, this, &IqtaxiGui::onFreqDebounceTimeout);

    setupUi();
    resetToDefaults();
    applyDiscovery(true);
    if (!m_sampleSource) {
        setStatus("Sample source unavailable");
        m_startStopButton->setEnabled(false);
    }

    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()), Qt::QueuedConnection);
    if (m_sampleSource) {
        m_sampleSource->setMessageQueueToGUI(&m_inputMessageQueue);
    }

    connect(&m_statusTimer, &QTimer::timeout, this, &IqtaxiGui::updateStatus);
    m_statusTimer.start(500);
}

IqtaxiGui::~IqtaxiGui()
{
    m_freqDebounceTimer.stop();
    m_statusTimer.stop();
}

void IqtaxiGui::resetToDefaults()
{
    const std::string model = m_settings.device_model;
    m_settings = IqtaxiSettings{};
    m_settings.device_model = model.empty() ? "E206" : model;
    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(m_settings.device_model))
    {
        m_settings.sample_rate_hz = caps->default_sample_rate_hz;
        m_settings.rx_gain = iqtaxiClampGain(m_settings.device_model, 30u);
    }
    displaySettings();
    applyAndSendSettings(true);
}

QByteArray IqtaxiGui::serialize() const
{
    if (m_sampleSource)
    {
        return m_sampleSource->serialize();
    }
    return QByteArray{};
}

bool IqtaxiGui::deserialize(const QByteArray &data)
{
    if (!m_sampleSource)
    {
        setStatus("deserialize skipped: no sample source");
        return false;
    }

    const bool ok = m_sampleSource->deserialize(data);
    if (ok)
    {
        // Push local UI settings to device once after restore.
        applyAndSendSettings(true);
    }
    return ok;
}

void IqtaxiGui::populateSampleRates()
{
    m_rateCombo->clear();
    const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(m_settings.device_model);
    if (!caps)
    {
        return;
    }
    for (std::size_t i = 0; i < caps->sample_rate_count; ++i)
    {
        const uint32_t rate = caps->sample_rates[i];
        m_rateCombo->addItem(
            QString::fromLatin1(iqtaxiSampleRateLabel(rate)),
            QVariant::fromValue(static_cast<quint32>(rate)));
    }
}

void IqtaxiGui::setupUi()
{
    auto *mainLayout = new QVBoxLayout(m_rootWidget);
    const QString modelLabel = QString::fromStdString(m_settings.device_model);
    m_descLabel = new QLabel(QString("IQTAXI %1").arg(modelLabel), m_rootWidget);
    mainLayout->addWidget(m_descLabel);

    auto *form = new QFormLayout();
    m_ipEdit = new QLineEdit(m_rootWidget);
    m_scanButton = new QPushButton("Scan", m_rootWidget);
    m_scanButton->setToolTip("UDP discover MicroPhase boards, fill Device IP, and show E100-6G/10G");
    auto *ipRow = new QWidget(m_rootWidget);
    auto *ipLayout = new QHBoxLayout(ipRow);
    ipLayout->setContentsMargins(0, 0, 0, 0);
    ipLayout->addWidget(m_ipEdit);
    ipLayout->addWidget(m_scanButton);
    m_freqDial = new ValueDial(m_rootWidget, ColorMapper(ColorMapper::GrayGold));
    // ValueDial only gets a usable size after setFont() (sets fixed width/height).
    {
        QFont dialFont("Liberation Mono");
        dialFont.setPointSize(16);
        m_freqDial->setFont(dialFont);
    }
    // Same as the tested ref/E100 plugin: dial covers 10 GHz. 6G boards are
    // identified after start via firmware RF-range / BOARD_BAND readback.
    const quint64 maxKhz =
        std::max<quint64>(1ull, iqtaxiMaxCenterFreqHz(m_settings.device_model) / 1000ull);
    m_freqDial->setValueRange(9, 1ull, maxKhz);
    m_freqDial->setToolTip("Center frequency (kHz). Scroll each digit with mouse wheel.");
    m_freqDial->setCursor(Qt::PointingHandCursor);
    m_freqDial->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);

    auto *freqRow = new QWidget(m_rootWidget);
    auto *freqLayout = new QHBoxLayout(freqRow);
    freqLayout->setContentsMargins(0, 0, 0, 0);
    freqLayout->addWidget(m_freqDial);
    freqLayout->addWidget(new QLabel("kHz", freqRow));
    freqLayout->addStretch(1);

    m_rateCombo = new QComboBox(m_rootWidget);
    populateSampleRates();
    m_rateCombo->setToolTip(QString("%1 supported sample rates").arg(modelLabel));
    m_gainSpin = new QSpinBox(m_rootWidget);
    if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(m_settings.device_model))
    {
        m_gainSpin->setRange(static_cast<int>(caps->gain_min), static_cast<int>(caps->gain_max));
    }
    else
    {
        m_gainSpin->setRange(0, 76);
    }
    form->addRow("Device IP", ipRow);
    form->addRow("Center Freq", freqRow);
    form->addRow("Sample Rate", m_rateCombo);
    form->addRow("RX Gain", m_gainSpin);
    mainLayout->addLayout(form);

    auto *btnLayout = new QHBoxLayout();
    // Same control as FileInput/USRP and workspace "Start all devices":
    // checkable ButtonSwitch with play/stop icons; webapiRun updates checked state.
    m_startStopButton = new ButtonSwitch(m_rootWidget);
    {
        QIcon icon;
        icon.addPixmap(QPixmap(":/play.png"), QIcon::Normal, QIcon::Off);
        icon.addPixmap(QPixmap(":/stop.png"), QIcon::Normal, QIcon::On);
        m_startStopButton->setIcon(icon);
    }
    m_startStopButton->setToolTip("Start/Stop acquisition (linked with workspace Start all devices)");
    m_startStopButton->setFixedSize(28, 28);
    m_startStopButton->setStyleSheet("QToolButton { background-color : blue; }");
    m_applyButton = new QPushButton("Apply", m_rootWidget);
    m_applyButton->setVisible(false);
    btnLayout->addWidget(m_startStopButton);
    btnLayout->addWidget(m_applyButton);
    btnLayout->addStretch(1);
    mainLayout->addLayout(btnLayout);

    connect(m_startStopButton, &ButtonSwitch::toggled, this, &IqtaxiGui::onStartStopToggled);
    connect(m_applyButton, &QPushButton::clicked, this, &IqtaxiGui::onApplyClicked);
    connect(m_scanButton, &QPushButton::clicked, this, &IqtaxiGui::onScanClicked);
    connect(m_ipEdit, &QLineEdit::editingFinished, this, &IqtaxiGui::onParamsEdited);
    connect(m_freqDial, &ValueDial::changed, this, &IqtaxiGui::onCenterFrequencyChanged);
    connect(m_rateCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &IqtaxiGui::onParamsEdited);
    connect(m_gainSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, &IqtaxiGui::onParamsEdited);

    auto *container = getContents();
    auto *containerLayout = new QVBoxLayout(container);
    containerLayout->addWidget(m_rootWidget);
    sizeToContents();
}

void IqtaxiGui::displaySettings()
{
    m_updatingUi = true;
    m_freqDebounceTimer.stop();
    m_ipEdit->setText(QString::fromStdString(m_settings.device_addr));
    m_freqDial->blockSignals(true);
    m_freqDial->setValue(m_settings.center_freq_hz / 1000ull);
    m_freqDial->blockSignals(false);
    m_pendingFreqKhz = m_settings.center_freq_hz / 1000ull;

    const int rateIndex = m_rateCombo->findData(
        QVariant::fromValue(static_cast<quint32>(m_settings.sample_rate_hz)));
    if (rateIndex >= 0)
    {
        m_rateCombo->setCurrentIndex(rateIndex);
    }
    else
    {
        if (const IqtaxiDeviceCaps *caps = iqtaxiDeviceCapsByModel(m_settings.device_model))
        {
            const int fallback = m_rateCombo->findData(
                QVariant::fromValue(static_cast<quint32>(caps->default_sample_rate_hz)));
            m_rateCombo->setCurrentIndex(fallback >= 0 ? fallback : 0);
        }
        else
        {
            m_rateCombo->setCurrentIndex(0);
        }
        m_settings.sample_rate_hz = m_rateCombo->currentData().toUInt();
    }

    m_gainSpin->setValue(static_cast<int>(m_settings.rx_gain));
    m_updatingUi = false;
}

void IqtaxiGui::applyAndSendSettings(bool force)
{
    if (m_updatingUi)
    {
        return;
    }
    m_freqDebounceTimer.stop();
    m_settings.device_addr = m_ipEdit->text().trimmed().toStdString();
    // ValueDial animates: getValue() is old until animation ends; USRP uses getValueNew().
    const uint64_t maxHz = iqtaxiMaxCenterFreqHz(m_settings.device_model);
    m_settings.center_freq_hz = std::min(static_cast<uint64_t>(m_freqDial->getValueNew() * 1000ull), maxHz);
    m_settings.sample_rate_hz = m_rateCombo->currentData().toUInt();
    m_settings.rx_gain = iqtaxiClampGain(
        m_settings.device_model, static_cast<uint32_t>(m_gainSpin->value()));

    if (!m_settings.is_valid() || !m_sampleSource)
    {
        return;
    }

    auto *msg = IqtaxiInput::MsgConfigureIqtaxi::create(m_settings, force);
    m_sampleSource->getInputMessageQueue()->push(msg);
}

bool IqtaxiGui::handleMessage(const Message &message)
{
    if (IqtaxiInput::MsgConfigureIqtaxi::match(message))
    {
        const auto &cfg = static_cast<const IqtaxiInput::MsgConfigureIqtaxi &>(message);
        m_settings = cfg.getSettings();
        displaySettings();
        return true;
    }
    if (IqtaxiInput::MsgStartStop::match(message))
    {
        // From workspace WebAPI run/stop: sync button without re-sending MsgStartStop.
        const auto &cmd = static_cast<const IqtaxiInput::MsgStartStop &>(message);
        setStartStopChecked(cmd.getStartStop());
        if (cmd.getStartStop() && !m_boardLabel.isEmpty())
        {
            setStatus(QString("running  %1").arg(m_boardLabel));
        }
        else
        {
            setStatus(cmd.getStartStop() ? "running" : "stopped");
        }
        return true;
    }
    if (IqtaxiInput::MsgReportError::match(message))
    {
        const auto &err = static_cast<const IqtaxiInput::MsgReportError &>(message);
        setStatus("error: " + err.getError());
        setStartStopChecked(false);
        return true;
    }
    if (IqtaxiInput::MsgReportRfBand::match(message))
    {
        const auto &report = static_cast<const IqtaxiInput::MsgReportRfBand &>(message);
        applyBoardLabel(report.getRfBand());
        return true;
    }
    return false;
}

void IqtaxiGui::applyBoardLabel(const QString &board)
{
    m_boardLabel = board.trimmed();
    if (m_boardLabel.isEmpty())
    {
        return;
    }

    setTitle(QString("IQTAXI %1").arg(m_boardLabel));
    if (m_descLabel)
    {
        m_descLabel->setText(QString("IQTAXI %1").arg(m_boardLabel));
    }

    if (m_freqDial && m_boardLabel.contains(QLatin1String("E100")))
    {
        const quint64 maxKhz = m_boardLabel.contains(QLatin1String("6G")) ? 6000000ull : 10000000ull;
        m_freqDial->setValueRange(9, 1ull, maxKhz);
    }
}

void IqtaxiGui::setStartStopChecked(bool checked)
{
    m_doApplySettings = false;
    m_startStopButton->blockSignals(true);
    m_startStopButton->setChecked(checked);
    m_startStopButton->blockSignals(false);
    m_doApplySettings = true;
}

void IqtaxiGui::onStartStopToggled(bool checked)
{
    if (!m_doApplySettings)
    {
        return;
    }
    if (!m_sampleSource)
    {
        setStatus("start/stop failed: no sample source");
        setStartStopChecked(false);
        return;
    }

    applyAndSendSettings(false);
    auto *msg = IqtaxiInput::MsgStartStop::create(checked);
    m_sampleSource->getInputMessageQueue()->push(msg);
    setStatus(checked ? "starting..." : "stopping...");
}

void IqtaxiGui::onApplyClicked()
{
    applyAndSendSettings(false);
    setStatus("settings applied");
}

void IqtaxiGui::onParamsEdited()
{
    applyAndSendSettings(false);
    setStatus("settings updated");
}

bool IqtaxiGui::applyDiscovery(bool fillIp)
{
    const auto found = sdr::api::iqtaxi_udp_discover(250);
    for (const auto &info : found)
    {
        if (info.name != m_settings.device_model)
        {
            continue;
        }
        if (fillIp && m_ipEdit)
        {
            m_ipEdit->setText(QString::fromStdString(info.addr));
        }
        const QString label = QString::fromStdString(
            sdr::api::iqtaxi_model_label(info.name, info.board_version));
        applyBoardLabel(label);
        applyAndSendSettings(false);
        setStatus(QString("found %1 serial %2 at %3")
                      .arg(label)
                      .arg(QString::fromStdString(info.serial))
                      .arg(QString::fromStdString(info.addr)));
        return true;
    }
    setStatus(found.empty()
                  ? "no UDP discovery response"
                  : QString("no %1 in discovery responses")
                        .arg(QString::fromStdString(m_settings.device_model)));
    return false;
}

void IqtaxiGui::onScanClicked()
{
    applyDiscovery(true);
}

void IqtaxiGui::onCenterFrequencyChanged(quint64 value)
{
    if (m_updatingUi)
    {
        return;
    }
    // Debounce wheel/digit changes: only the last value within the window is sent.
    m_pendingFreqKhz = value;
    m_settings.center_freq_hz = value * 1000ull;
    m_freqDebounceTimer.start(kFreqDebounceMs);
    setStatus(QString("freq %1 kHz (pending)").arg(value));
}

void IqtaxiGui::onFreqDebounceTimeout()
{
    if (m_updatingUi || !m_sampleSource)
    {
        return;
    }
    m_settings.center_freq_hz = m_pendingFreqKhz * 1000ull;
    m_settings.device_addr = m_ipEdit->text().trimmed().toStdString();
    m_settings.sample_rate_hz = m_rateCombo->currentData().toUInt();
    m_settings.rx_gain = static_cast<uint32_t>(m_gainSpin->value());

    if (!m_settings.is_valid())
    {
        return;
    }

    auto *msg = IqtaxiInput::MsgConfigureIqtaxi::create(m_settings, false);
    m_sampleSource->getInputMessageQueue()->push(msg);
    setStatus(QString("freq %1 kHz").arg(m_pendingFreqKhz));
}

void IqtaxiGui::handleInputMessages()
{
    Message *message;

    while ((message = m_inputMessageQueue.pop()) != nullptr)
    {
        (void)handleMessage(*message);
        delete message;
    }
}

void IqtaxiGui::updateStatus()
{
    if (!m_deviceUISet || !m_deviceUISet->m_deviceAPI)
    {
        return;
    }

    // Keep "freq xxx kHz (pending)" visible while debounce is active.
    if (m_freqDebounceTimer.isActive())
    {
        return;
    }

    const int state = m_deviceUISet->m_deviceAPI->state();
    if (state != m_lastEngineState)
    {
        switch (state)
        {
        case DeviceAPI::StNotStarted:
            m_startStopButton->setStyleSheet("QToolButton { background:rgb(79,79,79); }");
            break;
        case DeviceAPI::StIdle:
            m_startStopButton->setStyleSheet("QToolButton { background-color : blue; }");
            setStartStopChecked(false);
            break;
        case DeviceAPI::StRunning:
            m_startStopButton->setStyleSheet("QToolButton { background-color : green; }");
            setStartStopChecked(true);
            break;
        case DeviceAPI::StError:
            m_startStopButton->setStyleSheet("QToolButton { background-color : red; }");
            setStartStopChecked(false);
            break;
        default:
            break;
        }
        m_lastEngineState = state;
    }

    QString stateStr;
    m_deviceUISet->m_deviceAPI->getDeviceEngineStateStr(stateStr);
    if (state == DeviceAPI::StRunning && !m_boardLabel.isEmpty())
    {
        setStatus(QString("%1  %2").arg(stateStr, m_boardLabel));
    }
    else
    {
        setStatus(stateStr);
    }
}
