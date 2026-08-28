#pragma once

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QWidget>

#include <device/devicegui.h>
#include "gui/buttonswitch.h"
#include "gui/valuedial.h"

#include "iqtaxiinput.h"

class DeviceUISet;

class IqtaxiGui : public DeviceGUI
{
    Q_OBJECT
public:
    explicit IqtaxiGui(DeviceUISet *deviceUISet, QWidget *parent = nullptr);
    ~IqtaxiGui() override;

    void resetToDefaults() override;
    QByteArray serialize() const override;
    bool deserialize(const QByteArray &data) override;
    MessageQueue *getInputMessageQueue() override { return &m_inputMessageQueue; }

private:
    void setupUi();
    void populateSampleRates();
    void displaySettings();
    void applyAndSendSettings(bool force);
    bool handleMessage(const Message &message);
    void applyBoardLabel(const QString &board);
    void setStartStopChecked(bool checked);
    bool applyDiscovery(bool fillIp);

private slots:
    void onStartStopToggled(bool checked);
    void onApplyClicked();
    void onParamsEdited();
    void onScanClicked();
    void onCenterFrequencyChanged(quint64 value);
    void onFreqDebounceTimeout();
    void handleInputMessages();
    void updateStatus();

private:
    // Note: m_deviceUISet is inherited from DeviceGUI. Declaring it here again would
    // shadow it and DeviceGUI::setWorkspaceIndex() would never reach DeviceAPI,
    // breaking the workspace "Start all devices" button.
    DeviceSampleSource *m_sampleSource;
    IqtaxiSettings m_settings;

    QWidget *m_rootWidget;
    QLabel *m_descLabel;
    QLineEdit *m_ipEdit;
    QPushButton *m_scanButton;
    ValueDial *m_freqDial;
    QComboBox *m_rateCombo;
    QSpinBox *m_gainSpin;
    ButtonSwitch *m_startStopButton;
    QPushButton *m_applyButton;
    QTimer m_statusTimer;
    QTimer m_freqDebounceTimer;
    quint64 m_pendingFreqKhz;
    int m_lastEngineState;
    bool m_doApplySettings;
    bool m_updatingUi;
    QString m_boardLabel;

    MessageQueue m_inputMessageQueue;
};
