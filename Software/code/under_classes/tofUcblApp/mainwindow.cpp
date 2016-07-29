#include "mainwindow.h"
#include "ui_mainwindow.h"

//***********************************************************************************************************************************************
// * Destructors and Constructors
//***********************************************************************************************************************************************

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    valx(0.15),
    valy(0.25),
    valz(0.30),
    vali(0.01)
{
    ui->setupUi(this);

    ui->pushButton_capture->setEnabled(false);
    ui->pushButton_visu->setEnabled(false);

    if(mngCam.get_numb_connected_devices()>0)
    {
        ui->pushButton_capture->setEnabled(true);

        std::vector<std::string> list = mngCam.get_devices_name();
        profilmap = mngCam.get_profiles_name();

        for(int i=0; i<list.size(); i++)
            ui->comboBox_devices->addItem(QString::fromStdString(list[i]));

        typedef std::map<std::string, int>::iterator itmap_type;

        for(itmap_type iter = profilmap.begin(); iter != profilmap.end(); iter++)
        {
            ui->comboBox_calib->addItem(QString::fromStdString( iter->first ));
        }
    }

    //mngCam.get_cam_infos(ui->comboBox_devices->currentIndex());



    init_viewer();

    // Setup the cloud pointer
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);

    // The number of points in the cloud
    cloud->points.resize (200);

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    mngPcl.init_viewer(viewer);
    mngPcl.simpleVis(cloud);
    ui->widget_visu->update();

    create_actions();

    serial = new QSerialPort(this);
}

MainWindow::~MainWindow()
{
    delete ui;
    serial->close();
    delete serial;
}


//***********************************************************************************************************************************************
// * Functions
//***********************************************************************************************************************************************

void MainWindow::init_viewer()
{
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->widget_visu->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->widget_visu->GetInteractor(), ui->widget_visu->GetRenderWindow());
    ui->widget_visu->update();
}


void MainWindow::create_actions()
{
    // "New session" action (future implementation)
    descr = new QAction( tr("&Descriptions"), this);
    descr->setShortcut(tr("CTRL+D"));
    descr->setStatusTip(tr("Give descriptions of parameters"));
    descr->setEnabled(true);

    ui->menuParameters->addAction(descr);
    connect(descr, SIGNAL(triggered()), this, SLOT(display_descriptions()));

    //connect(ui->pushButton_stop, SIGNAL(clicked()), &mngPcl, SIGNAL(test()));


}

//***********************************************************************************************************************************************
// * Slots
//***********************************************************************************************************************************************

void MainWindow::display_descriptions()
{
    QWidget *winDescr = new QWidget();
    QListWidget *listDescr = new QListWidget(winDescr);
    listDescr->setGeometry(0, 0, 1000, 500);
    winDescr->setGeometry(0, 0, 1000, 500);
    //QLabel *labDescr = new QLabel(winDescr);
    std::string txtDescr;

    std::map<std::string, std::string> mapDescr = mngCam.get_param_descr(0);

    typedef std::map< std::string, std::string >::iterator it_type;
    for( it_type iter = mapDescr.begin(); iter != mapDescr.end(); iter++ )
    {
        txtDescr = iter->first + ": " + iter->second;

        listDescr->addItem(QString::fromStdString(txtDescr));
    }

    //labDescr->setText(QString::fromStdString(txtDescr));

    winDescr->show();

}

void MainWindow::on_pushButton_capture_clicked()
{

    //Get index of selected device
    mngCam.set_numOfShots(ui->spinBox_nShots->value());
    mngCam.set_freq(ui->doubleSpinBox_freq->value());
    int sel_device = ui->comboBox_devices->currentIndex();
    double freq = ui->doubleSpinBox_freq->value();
//    double period = double(1/freq)*1000000;
//    std::cout << period << std::endl;
    double period = 0;
    //Get index of camera calibration mode
    std::string sel_calib = ui->comboBox_calib->currentText().toStdString();
    int idCalib = profilmap[sel_calib];

    int n_clouds = ui->spinBox_nclouds->value();
    list_clouds = std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr >(n_clouds);
    ui->spinBox_visu->setMaximum(n_clouds-1);

    // Setup the cloud pointer
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
    for(int i = 0; i < n_clouds; i++)
    {
        list_clouds[i] = mngCam.capture(sel_device, idCalib);
        if(serial->isWritable())
            serial->write("r1r");
        else
            qDebug() << "Couldn't write to the serial";
        usleep(period);
    }
//    cloud = mngCam.capture(sel_device, idCalib);
    cloud = list_clouds[0];
    mngPcl.set_cloud(cloud);
    ui->pushButton_visu->setEnabled(true);
    ui->label_infos->setText("Captured");
    //cloud = mngPcl.filter_cloud(cloud);

    on_pushButton_filter_clicked();

}

void MainWindow::on_pushButton_visu_clicked()
{
    int ith_cloud = ui->spinBox_visu->value();
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
    cloud = list_clouds[ith_cloud];
    mngPcl.set_cloud(cloud);

    if(ui->radioButton_viewer->isChecked())
        mngPcl.visualizePcl();

    else if(ui->radioButton_vizualizer->isChecked())
    {
        viewer->removeAllPointClouds();
        viewer = mngPcl.simpleVis(cloud);
        ui->widget_visu->update();
    }
}


void MainWindow::on_pushButton_import_clicked()
{
    QStringList filenameList = QFileDialog::getOpenFileNames();
    int sz = filenameList.size();
    QString filename;

    if(filenameList.at(0).compare("")!=0)
    {
        list_clouds = std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr >(sz);
        ui->spinBox_visu->setMaximum(sz-1);

        for( int i = 0; i < sz; i++ )
        {
            filename = filenameList.at(i);

            cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
            mngPcl.set_cloud_from_pcd(filename.toStdString());


            list_clouds[i] = mngPcl.get_cloud();
        }

        cloud = list_clouds[0];
        ui->pushButton_visu->setEnabled(true);
    }
}

void MainWindow::on_pushButton_save_clicked()
{
    QString saveFileName = QFileDialog::getSaveFileName();
    int sz = list_clouds.size();
    QString nameEach;

    if( saveFileName.compare("")!=0 )
    {
        for( int i = 0; i < sz; i++ )
        {
            nameEach = saveFileName + QString::number(i) + ".pcd";
            mngPcl.set_cloud(list_clouds[i]);
            mngPcl.save2pcd(nameEach.toStdString());
        }
    }
}

void MainWindow::on_pushButton_filter_clicked()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    int sz = list_clouds.size();

    for( int i = 0; i < sz; i++)
    {
        filtered = mngPcl.filter_cloud(list_clouds[i], valx, valy, valz, vali);
        cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
        cloud = filtered;
        list_clouds[i] = cloud;
    }
    mngPcl.set_cloud(cloud);
}

void MainWindow::on_horizontalSlider_filterx_valueChanged(int value)
{
    valx = float(value)/100;
    ui->label_xv->setText(QString::number(valx));
}

void MainWindow::on_horizontalSlider_filtery_valueChanged(int value)
{
    valy = float(value)/100;
    ui->label_yv->setText(QString::number(valy));
}

void MainWindow::on_horizontalSlider_filterz_valueChanged(int value)
{
    valz = float(value)/100;
    ui->label_zv->setText(QString::number(valz));
}



void MainWindow::on_horizontalSlider_filteri_valueChanged(int value)
{
    vali = float(value)/10000;
    ui->label_iv->setText(QString::number(vali));
}


void MainWindow::on_pushButton_arduino_clicked()
{
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        if (serialPortInfo.productIdentifier() == 67)
                            arduino_port_name = serialPortInfo.portName();

        ui->label_infos->setText("Number of available ports: " + QString::number( QSerialPortInfo::availablePorts().length()) +
                            "\n" + "Vendor ID: " + QString::number(serialPortInfo.vendorIdentifier()) +
                            "\n" + "Product ID: " + QString::number(serialPortInfo.productIdentifier()) +
                            "\n" + "Arduino port name = "+ serialPortInfo.portName());
    }


    serial->setPortName(arduino_port_name);
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadWrite);
//    connect(serial,SIGNAL(readyRead()),this,SLOT(serialReader()));

    if(serial->isWritable())
        serial->write("r1r");
    else
        qDebug() << "Couldn't write to the serial";

}





