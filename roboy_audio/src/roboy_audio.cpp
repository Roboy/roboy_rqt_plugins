#include <roboy_audio/roboy_audio.hpp>



RoboyAudio::RoboyAudio()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyAudio");
}

void RoboyAudio::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    // create graph
    ui.avgLine_plot->addGraph();
    ui.avgLine_plot->graph(0)->setPen(QPen(color_pallette[1]));

    // fill ticks with 16 entries
    for(uint freq = 0; freq < NUMBER_OF_FREQUENCIES; freq++){
        ticks.push_back(freq);
    }

    // graph axes
    ui.avgLine_plot->xAxis->setLabel("frequencies");
    ui.avgLine_plot->yAxis->setLabel("yet unknown Unit");
    ui.avgLine_plot->replot();

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "audio_rqt_plugin");
    }

    micAudio = nh->subscribe("/roboy/cognition/audio/audio", 1, &RoboyAudio::MicAudio, this);
    avgPower = nh->subscribe("/roboy/cognition/audio/freqPower", 1, &RoboyAudio::AvgPower, this);
    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
}

void RoboyAudio::shutdownPlugin() {
    micAudio.shutdown();
}

void RoboyAudio::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyAudio::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyAudio::MicAudio(const roboy_communication_cognition::AudioData::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving audio signal");
}

void RoboyAudio::AvgPower(const roboy_communication_cognition::FreqPower::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving average frequency powers");
    time.push_back(counter++);

    freqLineData.clear();
    for (uint freq = 0; freq < msg->frequency.size(); freq++) {
        freqLineData.push_back(msg->averagePower[freq]);
    }


    if (time.size() > samples_per_plot)
        time.pop_front();

    Q_EMIT newData();
}

void RoboyAudio::plotData() {

    ui.avgLine_plot->graph(0)->setData(ticks, freqLineData);

    // keep the scale as big as needed
    for(double currPower : freqLineData) {
        if(currPower > maxPower){
            maxPower = currPower;
            ui.avgLine_plot->graph(0)->rescaleAxes();
        }
    }
    ui.avgLine_plot->replot();
}

PLUGINLIB_DECLARE_CLASS(roboy_audio, RoboyAudio, RoboyAudio, rqt_gui_cpp::Plugin)
