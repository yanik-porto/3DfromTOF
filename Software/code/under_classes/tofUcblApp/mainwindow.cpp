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

    //Disabled some functions before a camera is detected
    ui->pushButton_capture->setEnabled(false);
    ui->pushButton_visu->setEnabled(false);

    //Check how many cameras are connected
    if(mngCam.get_numb_connected_devices()>0)
    {
        //If at least 1 device is connected, enable the capture
        ui->pushButton_capture->setEnabled(true);

        //Get informations about the device
        std::vector<std::string> list = mngCam.get_devices_name();
        profilmap = mngCam.get_profiles_name();

        //Fill the GUI with the infos
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
    //Make the connection between a pcl visualizer and the widget in the gui
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->widget_visu->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->widget_visu->GetInteractor(), ui->widget_visu->GetRenderWindow());
    ui->widget_visu->update();
}


void MainWindow::create_actions()
{
    //Add buttons to the menu bar
    descr = new QAction( tr("&Descriptions"), this);
    descr->setShortcut(tr("CTRL+D"));
    descr->setStatusTip(tr("Give descriptions of parameters"));
    descr->setEnabled(true);

    ui->menuParameters->addAction(descr);
    connect(descr, SIGNAL(triggered()), this, SLOT(display_descriptions()));
}

//***********************************************************************************************************************************************
// * Slots
//***********************************************************************************************************************************************

void MainWindow::display_descriptions()
{
    //Display all the tunable parameters of the camera in the description menu
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

    winDescr->show();

}

void MainWindow::on_pushButton_capture_clicked()
{
    //Tell the user to wait til the end
    ui->label_infos->setText("Wait");

    //Get index of selected device and frequency
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

    //Initialize the vector of clouds with the number of desired clouds
    int n_clouds = ui->spinBox_nclouds->value();
    list_clouds = std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr >(n_clouds);
    ui->spinBox_visu->setMaximum(n_clouds-1);

    // Setup the cloud pointer
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
    for(int i = 0; i < n_clouds; i++)
    {
        //Call the capture function of the CameraManager for each cloud
        list_clouds[i] = mngCam.capture(sel_device, idCalib);

        //Send the rotation command to the Arduino Board
        if(serial->isWritable())
        {
            serial->flush();
            serial->write("r1r");
        }
        else
            qDebug() << "Couldn't write to the serial";
        usleep(period);

    }
//    cloud = mngCam.capture(sel_device, idCalib);
    //Prepare the cloud for visualization
    cloud = list_clouds[0];
    mngPcl.set_cloud(cloud);
    ui->pushButton_visu->setEnabled(true);
    ui->label_infos->setText("Captured");

    //Filter the cloud automatically with the values tuned in the gui
    on_pushButton_filter_clicked();

}

void MainWindow::on_pushButton_visu_clicked()
{
    //Get cloud index selected in the gui and initialize the cloud pointer
    int ith_cloud = ui->spinBox_visu->value();
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
    cloud = list_clouds[ith_cloud];
    mngPcl.set_cloud(cloud);

    //Visualize the cloud in the selected type of viewer
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
    //Get name of the folder to import
    QStringList filenameList = QFileDialog::getOpenFileNames();
    int sz = filenameList.size();
    QString filename;

    //Import if the name is not empty
    if(filenameList.at(0).compare("")!=0)
    {
        //Initialize the clouds vector with the number of files
        list_clouds = std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr >(sz);
        ui->spinBox_visu->setMaximum(sz-1);

        for( int i = 0; i < sz; i++ )
        {
            //Fill the vector with the current cloud (file).
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
    //Get save file name
    QString saveFileName = QFileDialog::getSaveFileName();
    int sz = list_clouds.size();
    QString nameEach;

    if( saveFileName.compare("")!=0 )
    {
        for( int i = 0; i < sz; i++ )
        {
            //Save all the captured clouds with the same name and a different number
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
        //Filter each cloud with the values indicated in the gui
        filtered = mngPcl.filter_cloud(list_clouds[i], valx, valy, valz, vali);
        cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
        cloud = filtered;
        list_clouds[i] = cloud;
    }
    mngPcl.set_cloud(cloud);
}

void MainWindow::on_horizontalSlider_filterx_valueChanged(int value)
{
    //Convert value of the slider into float
    valx = float(value)/100;
    ui->label_xv->setText(QString::number(valx));
}

void MainWindow::on_horizontalSlider_filtery_valueChanged(int value)
{
    //Convert value of the slider into float
    valy = float(value)/100;
    ui->label_yv->setText(QString::number(valy));
}

void MainWindow::on_horizontalSlider_filterz_valueChanged(int value)
{
    //Convert value of the slider into float
    valz = float(value)/100;
    ui->label_zv->setText(QString::number(valz));
}

void MainWindow::on_horizontalSlider_filteri_valueChanged(int value)
{
    //Convert value of the slider into float
    vali = float(value)/10000;
    ui->label_iv->setText(QString::number(vali));
}

void MainWindow::on_pushButton_arduino_clicked()
{
    //Check number of connected port
    foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        //Check if it is the arduino board
        if (serialPortInfo.productIdentifier() == 67)
                            arduino_port_name = serialPortInfo.portName();

        //Display informations of the board to the user
        ui->label_infos->setText("Number of available ports: " + QString::number( QSerialPortInfo::availablePorts().length()) +
                            "\n" + "Vendor ID: " + QString::number(serialPortInfo.vendorIdentifier()) +
                            "\n" + "Product ID: " + QString::number(serialPortInfo.productIdentifier()) +
                            "\n" + "Arduino port name = "+ serialPortInfo.portName());
    }

    //Set the port with the Arduino details
    serial->setPortName(arduino_port_name);
    serial->setBaudRate(QSerialPort::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->open(QIODevice::ReadWrite);
//    connect(serial,SIGNAL(readyRead()),this,SLOT(serialReader()));

    //Make a turn when connected
    if(serial->isWritable())
    {
        serial->flush();
        serial->write("r1r");
    }
    else
        qDebug() << "Couldn't write to the serial";

}


void MainWindow::on_pushButton_merge_clicked()
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_xyz(list_clouds.size());

    //Convert the xyzi cloud into xyz for passing into the fusion function
    for(int i = 0; i < list_clouds.size(); i++)
    {
        list_xyz[i] =  mngPcl.xyz_from_xyzi(list_clouds[i]);
    }

    //Pass the list of xyz clouds into the fusion
    ICP_Registration icpreg(list_xyz);

    //Initialize the vector of clouds with the number of desired clouds
    list_clouds = std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr >(1);
    ui->spinBox_visu->setMaximum(0);

    //Get the result of all merged clouds
    list_clouds[0] = icpreg.final;
    cloud = list_clouds[0];
}
