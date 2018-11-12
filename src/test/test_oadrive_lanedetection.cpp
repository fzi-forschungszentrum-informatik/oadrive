// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

#include <oadrive_lanedetection/RoadPatching/RoadPerception.h>
#include <oadrive_world/Environment.h>
#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_lanedetection/FeatureDetection/HaarFilter.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <chrono>


#include <oadrive_util/Config.h>
#include <opencv2/flann/timer.h>
#include <oadrive_util/BirdViewConverter.h>
#include <boost/make_shared.hpp>

using namespace std::chrono;

using namespace oadrive::core;
using namespace oadrive::lanedetection;
using namespace oadrive::util;
using namespace oadrive::world;

using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::world;

CoordinateConverter mConverter;

void showDebugImage()
{
  // Determines size of the objects in the image
  float pixelPerMeter = 100;

  cv::Size2f imageSize(800, 800);
  cv::Point2f imgCenter(400, 400);
  cv::Mat image(imageSize, CV_8UC4, cv::Scalar(0, 0, 0, 0));

  EnvironmentPtr env = Environment::getInstance();

  // ----------------- draw patches
  const PatchPtrList* street = env->getStreet();
  for (PatchPtr p : *street)
  {
    ExtendedPose2d localPose = mConverter.world2Car(env->getCarPose(), p->getPose());

    cv::Point2f center, rotatedCenter;
    center.x = localPose.getX() * pixelPerMeter + imgCenter.x;
    center.y = -localPose.getY() * pixelPerMeter + imgCenter.y;

    cv::Size2f patchSize(p->getWidth() * pixelPerMeter, p->getLength() * pixelPerMeter);

    cv::Scalar col(255, 0, 0);

    cv::RotatedRect rRect = cv::RotatedRect(center, patchSize,
                                            -localPose.getYaw() * 180.f / M_PI - 90);

    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
    {
      cv::line(image, vertices[i], vertices[(i + 1) % 4], col, 1);
    }

    cv::circle(image, center, 3, col, 1);
  }

  // ----------------- draw trajectory
  std::vector<oadrive::core::Trajectory2d> trajectories = env->getMultiTrajectory().trajectories;
  if (trajectories.size() > 0)
  {
    oadrive::core::Trajectory2d trajectory = trajectories.at(0);
    for (size_t i = 0; i < trajectory.size(); i++)
    {
      ExtendedPose2d localPose = mConverter.world2Car(env->getCarPose(), trajectory.at(i));

      cv::Point2f pos;
      pos.x = localPose.getX() * pixelPerMeter + imgCenter.x;
      pos.y = -localPose.getY() * pixelPerMeter + imgCenter.y;

      cv::Scalar col(0, 255, 0);
      cv::circle(image, pos, 4, col, 2);
    }
  }

  // ----------------- draw car
  cv::Point2f pos;
  pos.x = 0 * pixelPerMeter + imgCenter.x;
  pos.y = 0 * pixelPerMeter + imgCenter.y;
  cv::Scalar col(255, 255, 255);

  cv::Size2f carSize(env->getCar()->getWidth() * pixelPerMeter,
                     env->getCar()->getLength() * pixelPerMeter);
  cv::RotatedRect rRect =
          cv::RotatedRect(pos, carSize, 90.f);

  cv::Point2f vertices[4];
  rRect.points(vertices);
  for (int i = 0; i < 4; i++)
  {
    cv::line(image, vertices[i], vertices[(i + 1) % 4], col, 1);
  }
  cv::circle(image, pos, 2, col, 1);

  // ----------------- draw execution times
//  std::ostringstream oss;
//  pos.x = 10;
//  pos.y = 20;
//  oss.clear(); oss.str(""); oss << "Add Patches: " << mExecTimeAddPatches;
//  cv::putText( image, oss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255, 0));
//
//  pos.y = 40;
//  oss.clear(); oss.str(""); oss << "Generate Traj: " << mExecTimeGenTraj;
//  cv::putText( image, oss.str(), pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255, 0));


  // ----------------- display debug image
  imshow("environment", image);
  cv::waitKey(1);
}


int main(int argc, char** argv)
{

  if (argc < 4) {
    std::cout << "Usage: test_oadrive_lanedetection <folder with recordings> <configFolder> <car>" << std::endl;
    return 0;
  }

  std::string basePath(argv[1]);
  std::string configFolder(argv[2]);
  std::string car(argv[3]);

  std::string directory = "Intersection02";

  std::vector<std::string> filenames;
  std::vector<std::string> filenamesNet;
  for (int count = 0; count < 700; count++)
  {
    std::string filename = std::to_string(count);
    std::string tmpPrefix = "";
    while (tmpPrefix.size() < 6 - filename.size())
    {
      tmpPrefix.push_back('0');
    }


    filenames.push_back(basePath + "/" + tmpPrefix + filename + ".png");
    filenamesNet.push_back(basePath + "/net" + tmpPrefix + filename + ".png");
  }


  Config::setConfigPath(configFolder, car);

  BirdViewConverter birdView;
  birdView.loadConfig(configFolder + "/" + car + "/BirdviewCal.yml");
  mConverter = CoordinateConverter(configFolder + "/" + car + "/BirdviewCal.yml");

  cvflann::StartStopTimer timer;

  RoadPerception patcher(mConverter.getImgSizeBirdView(), &mConverter, true);
  Environment::init();
  patcher.setEstimatedFirstPatchPose(ExtendedPose2d(1.0, 0.25f, 0.f));

  TrajectoryFactory trajFactory;

  std::vector<OadrivePose> carPoses;

  // Check if there is a carPose.txt in the folder. If so, open it and read the car poses:
  std::ifstream file(basePath + "/carPose.txt");
  if (file.is_open())
  {
    std::cout << "Found car pose file!" << std::endl;
    float x, y, yaw;
    std::string line;
    while (std::getline(file, line))
    {
      int firstOcc = line.find(':');
      int secondOcc = line.find(':', firstOcc + 1);

      x = std::stof(line.substr(0, firstOcc));
      y = std::stof(line.substr(firstOcc + 1, secondOcc));
      yaw = std::stof(line.substr(secondOcc + 1, line.size()));

      carPoses.push_back(OadrivePose(x, y, yaw));
    }
  }
  else
  {
    std::cout << "WARNING: Car pose file not found." << std::endl
              << "\tWill use 0,0,0 as car pose for every frame." << std::endl;
  }

  bool animate = false;

  float yaw = 0.f;

//  cv::Mat newBirdviewImage = cv::imread(filenames[4], CV_LOAD_IMAGE_GRAYSCALE );
//  cv::Mat newImageNet = cv::imread(filenamesNet[4], CV_LOAD_IMAGE_GRAYSCALE );
  /*********************** Loop ******************************/
  for (size_t i = 0; i < filenames.size(); ++i)
  {
//    yaw += 0.01;
//    OadrivePose carPose(5.f, 5.f, yaw);
    OadrivePose carPose(0.f, 0.f, 0.f);
    if (carPoses.size() > i)
    {
      carPose = carPoses[i];
    }


//    ExtendedPose2d carPoseTmp(0.f, 0.f, yaw);
    ExtendedPose2d carPoseTmp(carPose.getX(), carPose.getY(), carPose.getYaw());
    Environment::getInstance()->updateCarPose(carPoseTmp);

//    std::cout << filenames[i] << std::endl;
//    std::cout << "Car pose: " << carPose.getX() << " " << carPose.getY() << " " << carPose.getYaw()
//              << std::endl;

    cv::Mat newBirdviewImage = cv::imread(filenames[i], CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat newImageNet = cv::imread(filenamesNet[i], CV_LOAD_IMAGE_GRAYSCALE);
    std::cout << filenames[i] << std::endl;
    std::cout << filenamesNet[i] << std::endl;
    if (!newBirdviewImage.data || !newImageNet.data)
    {
      continue;
    }


    ExtendedPose2d globalPose = ExtendedPose2d(2.358118, 7.239433, -1.386257);
    TrafficSign sign(GIVE_WAY, globalPose);
    patcher.updateCarPose(carPose);
    patcher.executeFeatureDetection(newBirdviewImage);
    patcher.executeRoadPerception(newImageNet, &sign);


    for (PatchHypothesis patchHyp : patcher.getRoad())
    {
      PatchType type;
      drivingDirection drivDir;

      switch (patchHyp.getPatchType())
      {
        case PatchHypothesis::Type::ROAD:
          type = STRAIGHT;
          break;
        case PatchHypothesis::Type::INTERSECTION:
          type = CROSS_SECTION;
          break;
        case PatchHypothesis::Type::PARKING:
          type = PARKING;
          break;
        default:
          type = STRAIGHT;
      }

      switch (patchHyp.getDrivingDirection())
      {
        case PatchHypothesis::DrivingDirection::STRAIGHT:
          drivDir = oadrive::world::drivingDirection::DD_STRAIGHT;

          break;
        case PatchHypothesis::DrivingDirection::LEFT:
          drivDir = oadrive::world::drivingDirection::DD_LEFT;
          patchHyp.setYaw(patchHyp.getYaw() - M_PI_2);
          break;
        case PatchHypothesis::DrivingDirection::RIGHT:
          drivDir = oadrive::world::drivingDirection::DD_RIGHT;
          patchHyp.setYaw(patchHyp.getYaw() + M_PI_2);
          break;
      }

      ExtendedPose2d tmp(patchHyp.getPose().getX(), patchHyp.getPose().getY(),
                         patchHyp.getPose().getYaw());
      tmp = mConverter.car2World(carPoseTmp, tmp);

      PatchPtr p = boost::make_shared<Patch>(type, tmp);
      p->setProbability(patchHyp.getProbability());
      p->setPatchID(patchHyp.getPatchID());
      p->setAction(drivDir);

      Environment::getInstance()->addPatch(p);
    }

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    Environment::getInstance()->generateNextTrajectory();
    high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    float duration = duration_cast<milliseconds>(t2 - t1).count();
    LOGGING_WARNING(lanedetectionLogger, "-------------------------------------------------: " << endl);
    showDebugImage();

    cv::Mat features = patcher.generateDebugImage();
    cv::Mat birdViewImageRGB;
    cv::cvtColor(newBirdviewImage, birdViewImageRGB, CV_GRAY2BGR);
    cv::Mat channels[4];
    cv::split(features, channels);
    cv::Mat sum;
    cv::bitwise_or(birdViewImageRGB, 0, sum, 255 - channels[3]);
    cv::Mat featuresRGB;
    cv::cvtColor(features, featuresRGB, CV_BGRA2BGR);
    sum = sum + featuresRGB;
    imshow("output", sum);
    imshow("net", newImageNet);
    cv::waitKey(1);

    int code;
    if (animate)
    {
      code = cv::waitKey(1);
    }
    else
    {
      code = cv::waitKey(0);
    }

    if (code == 27)
    {    // ESC to abort
      return 1;
    }
    else if (code == 65 || code == 97)
    {
      animate = !animate;
    }

  }

}
