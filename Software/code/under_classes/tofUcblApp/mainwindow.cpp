#include "mainwindow.h"
#include "ui_mainwindow.h"

//***********************************************************************************************************************************************
// * Destructors and Constructors
//***********************************************************************************************************************************************

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->pushButton_visu->setEnabled(false);

    std::vector<std::string> list = mngCam.get_devices_name();
    profilmap = mngCam.get_profiles_name();

    for(int i=0; i<list.size(); i++)
        ui->comboBox_devices->addItem(QString::fromStdString(list[i]));

    typedef std::map<std::string, int>::iterator itmap_type;

    for(itmap_type iter = profilmap.begin(); iter != profilmap.end(); iter++)
    {
        ui->comboBox_calib->addItem(QString::fromStdString( iter->first ));
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

}

MainWindow::~MainWindow()
{
    delete ui;
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
//    connect(newAct, SIGNAL(triggered()), this, SLOT());
    descr->setEnabled(true);

    ui->menuParameters->addAction(descr);
    connect(descr, SIGNAL(triggered()), this, SLOT(display_descriptions()));




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
    int sel_device = ui->comboBox_devices->currentIndex();

    //Get index of camera calibration mode
    std::string sel_calib = ui->comboBox_calib->currentText().toStdString();
    int idCalib = profilmap[sel_calib];

    // Setup the cloud pointer
    cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
    cloud = mngCam.capture(sel_device, idCalib);
    mngPcl.set_cloud(cloud);
    ui->pushButton_visu->setEnabled(true);
    ui->label_infos->setText("Captured");
    //cloud = mngPcl.filter_cloud(cloud);

}

void MainWindow::on_pushButton_visu_clicked()
{

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
    QString filename = QFileDialog::getOpenFileName();
    if(filename.compare("")!=0)
    {
        cloud.reset (new pcl::PointCloud<pcl::PointXYZI>);
        mngPcl.set_cloud_from_pcd(filename.toStdString());
    }
}

void MainWindow::on_pushButton_save_clicked()
{
    QString saveFileName = QFileDialog::getSaveFileName();

    if( saveFileName.compare("")!=0 )
        mngPcl.save2pcd(saveFileName.toStdString());
}
