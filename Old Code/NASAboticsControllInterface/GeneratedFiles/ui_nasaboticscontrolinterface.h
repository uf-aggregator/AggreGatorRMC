/********************************************************************************
** Form generated from reading UI file 'nasaboticscontrolinterface.ui'
**
** Created by: Qt User Interface Compiler version 5.2.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NASABOTICSCONTROLINTERFACE_H
#define UI_NASABOTICSCONTROLINTERFACE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_NASAboticsControlInterfaceClass
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *bucket_pitch_control_layout_;
    QLabel *label_8;
    QHBoxLayout *horizontalLayout_6;
    QSlider *bucket_pitch_control_slider_;
    QSpinBox *bucket_pitch_control_spinbox_;
    QVBoxLayout *front_left_wheel_layout_;
    QLabel *label_3;
    QHBoxLayout *horizontalLayout;
    QSpinBox *front_left_wheel_spinbox_;
    QSlider *front_left_wheel_slider_;
    QFormLayout *controller_select_layout_;
    QLabel *controller_select_label_;
    QComboBox *controller_select_;
    QFormLayout *send_frequency_layout_;
    QLabel *send_frequency_label_;
    QDoubleSpinBox *send_frequency_spinbox_;
    QVBoxLayout *back_right_wheel_layout_;
    QLabel *label_6;
    QHBoxLayout *horizontalLayout_4;
    QSlider *back_right_wheel_slider_;
    QSpinBox *back_right_wheel_spinbox_;
    QVBoxLayout *front_right_wheel_layout_;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_2;
    QSlider *front_right_wheel_slider_;
    QSpinBox *front_right_wheel_spinbox_;
    QVBoxLayout *mine_control_layout_;
    QLabel *label_7;
    QHBoxLayout *horizontalLayout_5;
    QSpinBox *bucket_mine_control_spinbox_;
    QSlider *bucket_mine_control_slider_;
    QVBoxLayout *back_left_wheel_layout_;
    QLabel *label_5;
    QHBoxLayout *horizontalLayout_3;
    QSpinBox *back_left_wheel_spinbox_;
    QSlider *back_left_wheel_slider_;
    QVBoxLayout *verticalLayout;
    QPushButton *take_picture_button_;
    QPushButton *poll_ladar_button_;
    QLabel *label;
    QPushButton *connect_to_odroid_btn_;
    QPushButton *connect_to_launchpad_btn_;

    void setupUi(QWidget *NASAboticsControlInterfaceClass)
    {
        if (NASAboticsControlInterfaceClass->objectName().isEmpty())
            NASAboticsControlInterfaceClass->setObjectName(QStringLiteral("NASAboticsControlInterfaceClass"));
        NASAboticsControlInterfaceClass->resize(638, 286);
        gridLayout = new QGridLayout(NASAboticsControlInterfaceClass);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        bucket_pitch_control_layout_ = new QVBoxLayout();
        bucket_pitch_control_layout_->setSpacing(6);
        bucket_pitch_control_layout_->setObjectName(QStringLiteral("bucket_pitch_control_layout_"));
        label_8 = new QLabel(NASAboticsControlInterfaceClass);
        label_8->setObjectName(QStringLiteral("label_8"));

        bucket_pitch_control_layout_->addWidget(label_8);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        bucket_pitch_control_slider_ = new QSlider(NASAboticsControlInterfaceClass);
        bucket_pitch_control_slider_->setObjectName(QStringLiteral("bucket_pitch_control_slider_"));
        bucket_pitch_control_slider_->setMinimum(-128);
        bucket_pitch_control_slider_->setMaximum(127);
        bucket_pitch_control_slider_->setOrientation(Qt::Vertical);

        horizontalLayout_6->addWidget(bucket_pitch_control_slider_);

        bucket_pitch_control_spinbox_ = new QSpinBox(NASAboticsControlInterfaceClass);
        bucket_pitch_control_spinbox_->setObjectName(QStringLiteral("bucket_pitch_control_spinbox_"));
        bucket_pitch_control_spinbox_->setMinimum(-128);
        bucket_pitch_control_spinbox_->setMaximum(127);

        horizontalLayout_6->addWidget(bucket_pitch_control_spinbox_);


        bucket_pitch_control_layout_->addLayout(horizontalLayout_6);


        gridLayout->addLayout(bucket_pitch_control_layout_, 3, 0, 1, 1);

        front_left_wheel_layout_ = new QVBoxLayout();
        front_left_wheel_layout_->setSpacing(6);
        front_left_wheel_layout_->setObjectName(QStringLiteral("front_left_wheel_layout_"));
        label_3 = new QLabel(NASAboticsControlInterfaceClass);
        label_3->setObjectName(QStringLiteral("label_3"));

        front_left_wheel_layout_->addWidget(label_3);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        front_left_wheel_spinbox_ = new QSpinBox(NASAboticsControlInterfaceClass);
        front_left_wheel_spinbox_->setObjectName(QStringLiteral("front_left_wheel_spinbox_"));
        front_left_wheel_spinbox_->setMinimum(-32678);
        front_left_wheel_spinbox_->setMaximum(32678);
        front_left_wheel_spinbox_->setValue(0);

        horizontalLayout->addWidget(front_left_wheel_spinbox_);

        front_left_wheel_slider_ = new QSlider(NASAboticsControlInterfaceClass);
        front_left_wheel_slider_->setObjectName(QStringLiteral("front_left_wheel_slider_"));
        front_left_wheel_slider_->setMinimum(-32678);
        front_left_wheel_slider_->setMaximum(32677);
        front_left_wheel_slider_->setOrientation(Qt::Vertical);

        horizontalLayout->addWidget(front_left_wheel_slider_);


        front_left_wheel_layout_->addLayout(horizontalLayout);


        gridLayout->addLayout(front_left_wheel_layout_, 3, 1, 1, 1);

        controller_select_layout_ = new QFormLayout();
        controller_select_layout_->setSpacing(6);
        controller_select_layout_->setObjectName(QStringLiteral("controller_select_layout_"));
        controller_select_layout_->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        controller_select_label_ = new QLabel(NASAboticsControlInterfaceClass);
        controller_select_label_->setObjectName(QStringLiteral("controller_select_label_"));

        controller_select_layout_->setWidget(0, QFormLayout::LabelRole, controller_select_label_);

        controller_select_ = new QComboBox(NASAboticsControlInterfaceClass);
        controller_select_->setObjectName(QStringLiteral("controller_select_"));
        controller_select_->setEditable(false);

        controller_select_layout_->setWidget(0, QFormLayout::FieldRole, controller_select_);


        gridLayout->addLayout(controller_select_layout_, 0, 3, 1, 1);

        send_frequency_layout_ = new QFormLayout();
        send_frequency_layout_->setSpacing(6);
        send_frequency_layout_->setObjectName(QStringLiteral("send_frequency_layout_"));
        send_frequency_layout_->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        send_frequency_label_ = new QLabel(NASAboticsControlInterfaceClass);
        send_frequency_label_->setObjectName(QStringLiteral("send_frequency_label_"));

        send_frequency_layout_->setWidget(0, QFormLayout::LabelRole, send_frequency_label_);

        send_frequency_spinbox_ = new QDoubleSpinBox(NASAboticsControlInterfaceClass);
        send_frequency_spinbox_->setObjectName(QStringLiteral("send_frequency_spinbox_"));
        send_frequency_spinbox_->setMaximum(100);
        send_frequency_spinbox_->setValue(0.1);

        send_frequency_layout_->setWidget(0, QFormLayout::FieldRole, send_frequency_spinbox_);


        gridLayout->addLayout(send_frequency_layout_, 0, 0, 1, 1);

        back_right_wheel_layout_ = new QVBoxLayout();
        back_right_wheel_layout_->setSpacing(6);
        back_right_wheel_layout_->setObjectName(QStringLiteral("back_right_wheel_layout_"));
        label_6 = new QLabel(NASAboticsControlInterfaceClass);
        label_6->setObjectName(QStringLiteral("label_6"));

        back_right_wheel_layout_->addWidget(label_6);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        back_right_wheel_slider_ = new QSlider(NASAboticsControlInterfaceClass);
        back_right_wheel_slider_->setObjectName(QStringLiteral("back_right_wheel_slider_"));
        back_right_wheel_slider_->setMinimum(-32678);
        back_right_wheel_slider_->setMaximum(32677);
        back_right_wheel_slider_->setOrientation(Qt::Vertical);

        horizontalLayout_4->addWidget(back_right_wheel_slider_);

        back_right_wheel_spinbox_ = new QSpinBox(NASAboticsControlInterfaceClass);
        back_right_wheel_spinbox_->setObjectName(QStringLiteral("back_right_wheel_spinbox_"));
        back_right_wheel_spinbox_->setMinimum(-32678);
        back_right_wheel_spinbox_->setMaximum(32678);
        back_right_wheel_spinbox_->setValue(0);

        horizontalLayout_4->addWidget(back_right_wheel_spinbox_);


        back_right_wheel_layout_->addLayout(horizontalLayout_4);


        gridLayout->addLayout(back_right_wheel_layout_, 4, 2, 1, 1);

        front_right_wheel_layout_ = new QVBoxLayout();
        front_right_wheel_layout_->setSpacing(6);
        front_right_wheel_layout_->setObjectName(QStringLiteral("front_right_wheel_layout_"));
        label_4 = new QLabel(NASAboticsControlInterfaceClass);
        label_4->setObjectName(QStringLiteral("label_4"));

        front_right_wheel_layout_->addWidget(label_4);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        front_right_wheel_slider_ = new QSlider(NASAboticsControlInterfaceClass);
        front_right_wheel_slider_->setObjectName(QStringLiteral("front_right_wheel_slider_"));
        front_right_wheel_slider_->setMinimum(-32678);
        front_right_wheel_slider_->setMaximum(32677);
        front_right_wheel_slider_->setOrientation(Qt::Vertical);

        horizontalLayout_2->addWidget(front_right_wheel_slider_);

        front_right_wheel_spinbox_ = new QSpinBox(NASAboticsControlInterfaceClass);
        front_right_wheel_spinbox_->setObjectName(QStringLiteral("front_right_wheel_spinbox_"));
        front_right_wheel_spinbox_->setMinimum(-32678);
        front_right_wheel_spinbox_->setMaximum(32678);
        front_right_wheel_spinbox_->setValue(0);

        horizontalLayout_2->addWidget(front_right_wheel_spinbox_);


        front_right_wheel_layout_->addLayout(horizontalLayout_2);


        gridLayout->addLayout(front_right_wheel_layout_, 3, 2, 1, 1);

        mine_control_layout_ = new QVBoxLayout();
        mine_control_layout_->setSpacing(6);
        mine_control_layout_->setObjectName(QStringLiteral("mine_control_layout_"));
        label_7 = new QLabel(NASAboticsControlInterfaceClass);
        label_7->setObjectName(QStringLiteral("label_7"));

        mine_control_layout_->addWidget(label_7);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        bucket_mine_control_spinbox_ = new QSpinBox(NASAboticsControlInterfaceClass);
        bucket_mine_control_spinbox_->setObjectName(QStringLiteral("bucket_mine_control_spinbox_"));
        bucket_mine_control_spinbox_->setMinimum(-128);
        bucket_mine_control_spinbox_->setMaximum(127);

        horizontalLayout_5->addWidget(bucket_mine_control_spinbox_);

        bucket_mine_control_slider_ = new QSlider(NASAboticsControlInterfaceClass);
        bucket_mine_control_slider_->setObjectName(QStringLiteral("bucket_mine_control_slider_"));
        bucket_mine_control_slider_->setMinimum(-128);
        bucket_mine_control_slider_->setMaximum(127);
        bucket_mine_control_slider_->setOrientation(Qt::Vertical);

        horizontalLayout_5->addWidget(bucket_mine_control_slider_);


        mine_control_layout_->addLayout(horizontalLayout_5);


        gridLayout->addLayout(mine_control_layout_, 3, 3, 1, 1);

        back_left_wheel_layout_ = new QVBoxLayout();
        back_left_wheel_layout_->setSpacing(6);
        back_left_wheel_layout_->setObjectName(QStringLiteral("back_left_wheel_layout_"));
        label_5 = new QLabel(NASAboticsControlInterfaceClass);
        label_5->setObjectName(QStringLiteral("label_5"));

        back_left_wheel_layout_->addWidget(label_5);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        back_left_wheel_spinbox_ = new QSpinBox(NASAboticsControlInterfaceClass);
        back_left_wheel_spinbox_->setObjectName(QStringLiteral("back_left_wheel_spinbox_"));
        back_left_wheel_spinbox_->setMinimum(-32678);
        back_left_wheel_spinbox_->setMaximum(32678);
        back_left_wheel_spinbox_->setValue(0);

        horizontalLayout_3->addWidget(back_left_wheel_spinbox_);

        back_left_wheel_slider_ = new QSlider(NASAboticsControlInterfaceClass);
        back_left_wheel_slider_->setObjectName(QStringLiteral("back_left_wheel_slider_"));
        back_left_wheel_slider_->setMinimum(-32678);
        back_left_wheel_slider_->setMaximum(32677);
        back_left_wheel_slider_->setOrientation(Qt::Vertical);

        horizontalLayout_3->addWidget(back_left_wheel_slider_);


        back_left_wheel_layout_->addLayout(horizontalLayout_3);


        gridLayout->addLayout(back_left_wheel_layout_, 4, 1, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        take_picture_button_ = new QPushButton(NASAboticsControlInterfaceClass);
        take_picture_button_->setObjectName(QStringLiteral("take_picture_button_"));

        verticalLayout->addWidget(take_picture_button_);

        poll_ladar_button_ = new QPushButton(NASAboticsControlInterfaceClass);
        poll_ladar_button_->setObjectName(QStringLiteral("poll_ladar_button_"));

        verticalLayout->addWidget(poll_ladar_button_);


        gridLayout->addLayout(verticalLayout, 4, 3, 1, 1);

        label = new QLabel(NASAboticsControlInterfaceClass);
        label->setObjectName(QStringLiteral("label"));
        label->setStyleSheet(QStringLiteral("image: url(:/NASAboticsControlInterface/Resources/GR_Logo.jpg);"));

        gridLayout->addWidget(label, 4, 0, 1, 1);

        connect_to_odroid_btn_ = new QPushButton(NASAboticsControlInterfaceClass);
        connect_to_odroid_btn_->setObjectName(QStringLiteral("connect_to_odroid_btn_"));

        gridLayout->addWidget(connect_to_odroid_btn_, 0, 1, 1, 1);

        connect_to_launchpad_btn_ = new QPushButton(NASAboticsControlInterfaceClass);
        connect_to_launchpad_btn_->setObjectName(QStringLiteral("connect_to_launchpad_btn_"));

        gridLayout->addWidget(connect_to_launchpad_btn_, 0, 2, 1, 1);

        QWidget::setTabOrder(send_frequency_spinbox_, controller_select_);
        QWidget::setTabOrder(controller_select_, bucket_pitch_control_slider_);
        QWidget::setTabOrder(bucket_pitch_control_slider_, bucket_pitch_control_spinbox_);
        QWidget::setTabOrder(bucket_pitch_control_spinbox_, front_left_wheel_spinbox_);
        QWidget::setTabOrder(front_left_wheel_spinbox_, front_left_wheel_slider_);
        QWidget::setTabOrder(front_left_wheel_slider_, front_right_wheel_slider_);
        QWidget::setTabOrder(front_right_wheel_slider_, front_right_wheel_spinbox_);
        QWidget::setTabOrder(front_right_wheel_spinbox_, bucket_mine_control_spinbox_);
        QWidget::setTabOrder(bucket_mine_control_spinbox_, bucket_mine_control_slider_);
        QWidget::setTabOrder(bucket_mine_control_slider_, back_left_wheel_spinbox_);
        QWidget::setTabOrder(back_left_wheel_spinbox_, back_left_wheel_slider_);
        QWidget::setTabOrder(back_left_wheel_slider_, back_right_wheel_slider_);
        QWidget::setTabOrder(back_right_wheel_slider_, back_right_wheel_spinbox_);

        retranslateUi(NASAboticsControlInterfaceClass);
        QObject::connect(bucket_pitch_control_spinbox_, SIGNAL(valueChanged(int)), bucket_pitch_control_slider_, SLOT(setValue(int)));
        QObject::connect(bucket_pitch_control_slider_, SIGNAL(valueChanged(int)), bucket_pitch_control_spinbox_, SLOT(setValue(int)));
        QObject::connect(front_left_wheel_spinbox_, SIGNAL(valueChanged(int)), front_left_wheel_slider_, SLOT(setValue(int)));
        QObject::connect(front_left_wheel_slider_, SIGNAL(valueChanged(int)), front_left_wheel_spinbox_, SLOT(setValue(int)));
        QObject::connect(front_right_wheel_spinbox_, SIGNAL(valueChanged(int)), front_right_wheel_slider_, SLOT(setValue(int)));
        QObject::connect(front_right_wheel_slider_, SIGNAL(valueChanged(int)), front_right_wheel_spinbox_, SLOT(setValue(int)));
        QObject::connect(back_left_wheel_spinbox_, SIGNAL(valueChanged(int)), back_left_wheel_slider_, SLOT(setValue(int)));
        QObject::connect(back_left_wheel_slider_, SIGNAL(valueChanged(int)), back_left_wheel_spinbox_, SLOT(setValue(int)));
        QObject::connect(back_right_wheel_slider_, SIGNAL(valueChanged(int)), back_right_wheel_spinbox_, SLOT(setValue(int)));
        QObject::connect(back_right_wheel_spinbox_, SIGNAL(valueChanged(int)), back_right_wheel_slider_, SLOT(setValue(int)));
        QObject::connect(bucket_mine_control_spinbox_, SIGNAL(valueChanged(int)), bucket_mine_control_slider_, SLOT(setValue(int)));
        QObject::connect(bucket_mine_control_slider_, SIGNAL(valueChanged(int)), bucket_mine_control_spinbox_, SLOT(setValue(int)));

        QMetaObject::connectSlotsByName(NASAboticsControlInterfaceClass);
    } // setupUi

    void retranslateUi(QWidget *NASAboticsControlInterfaceClass)
    {
        NASAboticsControlInterfaceClass->setWindowTitle(QApplication::translate("NASAboticsControlInterfaceClass", "NASAboticsControlInterface", 0));
        label_8->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Bucket Pitch Control", 0));
        label_3->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Front Left Wheel", 0));
        controller_select_label_->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Controller:", 0));
        controller_select_->clear();
        controller_select_->insertItems(0, QStringList()
         << QApplication::translate("NASAboticsControlInterfaceClass", "Desktop", 0)
         << QApplication::translate("NASAboticsControlInterfaceClass", "XBox (type1)", 0)
        );
        send_frequency_label_->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Send Frequency: ", 0));
        label_6->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Back Right Wheel", 0));
        label_4->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Front Right Wheel", 0));
        label_7->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Bucket Mine Control", 0));
        label_5->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Back Left Wheel", 0));
        take_picture_button_->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Take Picture", 0));
        poll_ladar_button_->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Poll LADAR", 0));
        label->setText(QString());
        connect_to_odroid_btn_->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Connect to Odroid", 0));
        connect_to_launchpad_btn_->setText(QApplication::translate("NASAboticsControlInterfaceClass", "Connect to Launchpad", 0));
    } // retranslateUi

};

namespace Ui {
    class NASAboticsControlInterfaceClass: public Ui_NASAboticsControlInterfaceClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NASABOTICSCONTROLINTERFACE_H
