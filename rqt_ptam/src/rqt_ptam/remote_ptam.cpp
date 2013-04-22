/*

 Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <rqt_ptam/remote_ptam.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QMessageBox>
#include <QPainter>
#include <QKeyEvent>

namespace rqt_ptam {

RemotePTAM::RemotePTAM()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("RemotePTAM");
}

void RemotePTAM::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ui_.image_frame->installEventFilter(this);
//  ui_.horizontalLayout->installEventFilter(this);
//  ui_.horizontalLayout_2->installEventFilter(this);

  updateNamespaceList();
  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onNamespaceChanged(int)));

  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateNamespaceList()));

  ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
  connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this, SLOT(onZoom1(bool)));
  
  connect(ui_.subscribe_check_box, SIGNAL(toggled(bool)), this, SLOT(onSubscribe(bool)));

  connect(ui_.space_push_button, SIGNAL(pressed()), this, SLOT(onSpace()));

  connect(ui_.reset_push_button, SIGNAL(pressed()), this, SLOT(onReset()));

  std::cout<<"test\n";
}

bool RemotePTAM::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == ui_.image_frame && event->type() == QEvent::Paint)
  {
    QPainter painter(ui_.image_frame);
    if (!qimage_.isNull())
    {
      ui_.image_frame->resizeToFitAspectRatio();
      // TODO: check if full draw is really necessary
      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
      //painter.drawImage(paint_event->rect(), qimage_);
      qimage_mutex_.lock();
      painter.drawImage(ui_.image_frame->contentsRect(), qimage_);
      qimage_mutex_.unlock();
    } else {
      // default image with gradient
      QLinearGradient gradient(0, 0, ui_.image_frame->frameRect().width(), ui_.image_frame->frameRect().height());
      gradient.setColorAt(0, Qt::white);
      gradient.setColorAt(1, Qt::black);
      painter.setBrush(gradient);
      painter.drawRect(0, 0, ui_.image_frame->frameRect().width() + 1, ui_.image_frame->frameRect().height() + 1);
    }
    return false;
  }
  else if(event->type() == QEvent::KeyPress){
    QKeyEvent *key = static_cast<QKeyEvent *>(event);
    if(key->key() == Qt::Key_Space)
      onSpace();
    else if(key->key() == Qt::Key_R)
      onReset();
  }

  return QObject::eventFilter(watched, event);
}

void RemotePTAM::shutdownPlugin()
{
  subscriber_.shutdown();
  cmd_pub_.shutdown();
}

void RemotePTAM::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString topic = ui_.topics_combo_box->currentText();
  //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
  instance_settings.setValue("topic", topic);
  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  }

void RemotePTAM::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
  ui_.zoom_1_push_button->setChecked(zoom1_checked);

  QString topic = instance_settings.value("topic", "").toString();
  //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
  selectTopic(topic);
}

void RemotePTAM::updateNamespaceList()
{
  QSet<QString> message_types;
  QList<QString> transports;
  message_types.insert("ptam_com/ptam_info");

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopicList(message_types, transports);
  topics.append("");
  qSort(topics);
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {

    QString label(*it);
    label.replace(" ", "/");
    label.replace("/info", "");

//    std::cout<<"found "<<it->toStdString()<<" "<<label.toStdString()<<std::endl;
    ui_.topics_combo_box->addItem(label, QVariant(label));
  }

  // restore previous selection
  selectTopic(selected);
}

QList<QString> RemotePTAM::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QList<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();

      // add raw topic
      topics.append(topic);
      //qDebug("ImageView::getTopicList() raw topic '%s'", topic.toStdString().c_str());
      
      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.append(sub);
          //qDebug("ImageView::getTopicList() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
      }
    }
  }
  return topics;
}

void RemotePTAM::selectTopic(const QString& topic)
{
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1)
  {
    index = ui_.topics_combo_box->findText("");
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

void RemotePTAM::onNamespaceChanged(int index)
{
  subscriber_.shutdown();

  // reset image on topic change
  qimage_ = QImage();
  ui_.image_frame->update();

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();

  std::cout<<"topic"<<topic.toStdString()<<std::endl;

  ros::NodeHandle nh = getNodeHandle();

  if (!topic.isEmpty())
  {
//    cmd_pub_.shutdown();
    cmd_pub_ = nh.advertise<std_msgs::String>(topic.toStdString() + "/key_pressed", 10);
    image_transport::ImageTransport it(nh);
    try {
      subscriber_ = it.subscribe(topic.toStdString() + "/preview", 1, &RemotePTAM::callbackImage, this);
      //qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }
}

void RemotePTAM::onZoom1(bool checked)
{
  if (checked)
  {
    if (qimage_.isNull())
    {
      return;
    }
    ui_.image_frame->setInnerFrameFixedSize(qimage_.size());
    widget_->resize(ui_.image_frame->size());
    widget_->setMinimumSize(widget_->sizeHint());
    widget_->setMaximumSize(widget_->sizeHint());
  } else {
    ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
    ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
    widget_->setMinimumSize(QSize(80, 60));
    widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
  }
}

void RemotePTAM::onSubscribe(bool checked)
{
// TODO subscribe / unsubscribe
}

void RemotePTAM::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // First let cv_bridge do its magic
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    conversion_mat_ = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_WARN_STREAM("image conversion failed. Reason: "<< e.what());
  }

  // copy temporary image as it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);
  qimage_mutex_.lock();
  qimage_ = image.copy();
  qimage_mutex_.unlock();

  ui_.image_frame->setAspectRatio(qimage_.width(), qimage_.height());
  if (!ui_.zoom_1_push_button->isEnabled())
  {
    ui_.zoom_1_push_button->setEnabled(true);
    onZoom1(ui_.zoom_1_push_button->isChecked());
  }
  ui_.image_frame->update();
}

void RemotePTAM::onReset()
{
  std::cout << "Sending \"r\" to ptam" << std::endl;
  std_msgs::StringPtr msg(new std_msgs::String);
  msg->data = "r";
  cmd_pub_.publish(msg);
}

void RemotePTAM::onSpace()
{
  std::cout << "Sending \"Space\" to ptam" << std::endl;
  std_msgs::StringPtr msg(new std_msgs::String);
  msg->data = "Space";
  cmd_pub_.publish(msg);
}


}

PLUGINLIB_EXPORT_CLASS(rqt_ptam::RemotePTAM, rqt_gui_cpp::Plugin)
