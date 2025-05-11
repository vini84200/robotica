#include "PhiROS2Aria.hpp"

using std::placeholders::_1;

PhiROS2Aria::PhiROS2Aria() : Node("phi_aria_p3dx"),
                             conn(NULL), laserConnector(NULL), robot(NULL), serial_port(""), serial_baud(0), debug_aria(false),
                             sonar_enabled(true), publish_sonar(true), publish_aria_lasers(false),
                             myPublishCB(this, &PhiROS2Aria::publish)
// TicksMM(-1), DriftFactor(-99999), RevCount(-1),
{
    // read in runtime parameters

    // port and baud
    this->declare_parameter("port", "/dev/ttyUSB0");
    serial_port = this->get_parameter("port").as_string();
    RCLCPP_INFO(this->get_logger(), "Set port: [%s].", serial_port.c_str());

    this->declare_parameter("baud", 0);
    serial_baud = this->get_parameter("baud").as_int();
    if (serial_baud != 0)
        RCLCPP_INFO(this->get_logger(), "Set serial port baud rate: [%d].", serial_baud);

    // handle debugging more elegantly
    this->declare_parameter("debug_aria", false); // default not to debug
    debug_aria = this->get_parameter("debug_aria").as_bool();
    this->declare_parameter("aria_log_filename", "Aria.log");
    aria_log_filename = this->get_parameter("aria_log_filename").as_string();

    // whether to connect to lasers using aria
    this->declare_parameter("publish_aria_lasers", false);
    publish_aria_lasers = this->get_parameter("publish_aria_lasers").as_bool();

    // Get frame_ids to use.
    this->declare_parameter("odom_frame", "odom");
    frame_id_odom = this->get_parameter("odom_frame").as_string();
    this->declare_parameter("base_link_frame", "base_link");
    frame_id_base_link = this->get_parameter("base_link_frame").as_string();
    // this->declare_parameter("sonar_frame", "sonar");
    this->declare_parameter("sonar_frame", "base_link");
    frame_id_sonar = this->get_parameter("sonar_frame").as_string();
    // this->declare_parameter("laser_frame", "laser");
    this->declare_parameter("laser_frame", "base_link");
    frame_id_laser = this->get_parameter("laser_frame").as_string();

    // advertise services for data topics
    // second argument to advertise() is queue size.
    // other argmuments (optional) are callbacks, or a boolean "latch" flag (whether to send current data to new
    // subscribers when they subscribe).

    pose_pub = this->create_publisher<nav_msgs::msg::Odometry>("pose", 5);
    // absTruePose_pub = this->create_publisher<nav_msgs::msg::Odometry>("absTruePose", 5);
    // relTruePose_pub = this->create_publisher<nav_msgs::msg::Odometry>("relTruePose", 5);

    sonar_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("sonar", 5);
    laserScan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 5);
    laserCloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud", 5);

    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&PhiROS2Aria::cmdvel_cb, this, _1));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    veltime = this->now();
}

bool PhiROS2Aria::Setup()
{
    // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
    // called once per instance, and these objects need to persist until the process terminates.

    Aria::init();

    robot = new ArRobot();
    ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
    // Now add any parameters given via ros params (see RosAriaNode constructor):

    // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
    // for wireless serial connection. Otherwise, interpret it as a serial port name.
    size_t colon_pos = serial_port.find(":");
    RCLCPP_INFO(this->get_logger(), "serial_port: [%s].", serial_port.c_str());
    if (colon_pos != std::string::npos)
    {
        isSimulation_ = false;
        args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
        args->add(serial_port.substr(0, colon_pos).c_str());
        //    args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
        args->add("-remoteLaserTcpPort"); // pass robot's TCP port to Aria
        args->add(serial_port.substr(colon_pos + 1).c_str());
    }
    else
    {
        isSimulation_ = true;
        args->add("-rh %s", serial_port.c_str()); // pass robot's serial port to Aria
    }

    // if a baud rate was specified in baud parameter
    if (serial_baud != 0)
    {
        args->add("-robotBaud %d", serial_baud);
    }

    if (debug_aria)
    {
        // turn on all ARIA debugging
        args->add("-robotLogPacketsReceived");    // log received packets
        args->add("-robotLogPacketsSent");        // log sent packets
        args->add("-robotLogVelocitiesReceived"); // log received velocities
        args->add("-robotLogMovementSent");
        args->add("-robotLogMovementReceived");
        ArLog::init(ArLog::File, ArLog::Verbose, aria_log_filename.c_str(), true);
    }

    ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
                                                              //  argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
    argparser->log();

    // Connect to the robot
    conn = new ArRobotConnector(argparser, robot); // warning never freed
    if (!conn->connectRobot())
    {
        RCLCPP_INFO(this->get_logger(), "ARIA could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
        return false;
    }

    sick = new ArSick();

    if (publish_aria_lasers)
    {

        // create instance of SICK laser
        robot->addRangeDevice(sick);

        laserConnector = new ArLaserConnector(argparser, robot, conn);
        laserConnector->setupLaser(sick);
        sick->runAsync();
    }

    // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
    if (!Aria::parseArgs())
    {
        RCLCPP_INFO(this->get_logger(), "ARIA error parsing ARIA startup parameters!");
        return false;
    }

    //     robot->setHeading(0);

    // Enable the motors
    robot->enableMotors();

    // disable sonars on startup
    // robot->disableSonar();

    // Run ArRobot background processing thread
    robot->runAsync(true);

    // connect to lasers and create publishers
    if (publish_aria_lasers)
    {
        //      robot->lock();

        RCLCPP_INFO(this->get_logger(), "Connecting to laser(s) configured in ARIA parameter file(s)...");
        if (!laserConnector->connectLaser(sick))
        //    if (!laserConnector->connectLasers())
        {
            RCLCPP_INFO(this->get_logger(), "Error connecting to laser(s)...");
            return false;
        }

        laser = sick;
        int ln = 1;
        std::string tfname("laser");
        tfname += "_frame";
        RCLCPP_INFO(this->get_logger(), "Creating publisher for laser #%d named %s with tf frame name %s", ln, laser->getName(), tfname.c_str());

        laserScanMsg_.header.frame_id = frame_id_laser;
        laserScanMsg_.angle_min = ArMath::degToRad(-90);
        laserScanMsg_.angle_max = ArMath::degToRad(90);
        // laserScanMsg_.time_increment = ?
        laserScanMsg_.range_min = 0; // laser->getMinRange() / 1000.0;
        laserScanMsg_.range_max = laser->getMaxRange() / 1000.0;
        // laserCloudMsg_.header.frame_id = globaltfname;

        // Get angle_increment of the laser
        laserScanMsg_.angle_increment = 0;
        if (laser->canSetIncrement())
        {
            laserScanMsg_.angle_increment = laser->getIncrement();
        }
        else if (laser->getIncrementChoice() != NULL)
        {
            laserScanMsg_.angle_increment = laser->getIncrementChoiceDouble();
        }
        assert(laserScanMsg_.angle_increment > 0);
        laserScanMsg_.angle_increment *= M_PI / 180.0;

        RCLCPP_INFO(this->get_logger(), "Done creating laser publishers");
    }

    //   // subscribe to command topics
    //   cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
    //       boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));

    //   // register a watchdog for cmd_vel timeout
    //   double cmdvel_timeout_param = 0.6;
    //   n.param("cmd_vel_timeout", cmdvel_timeout_param, 0.6);
    //   cmdvel_timeout = ros::Duration(cmdvel_timeout_param);
    //   if (cmdvel_timeout_param > 0.0)
    //     cmdvel_watchdog_timer = n.createTimer(ros::Duration(0.1), &RosAriaNode::cmdvel_watchdog, this);

    // true pose handler (ONLY SIMULATION)
    if (isSimulation_)
    {
        simStatHandler_ = simStatPacketHandler;
        // Start gathering robot true pose every 100ms
        robot->addPacketHandler(&simStatHandler_, ArListPos::FIRST);
        robot->comInt(237, 2);
    }

    // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
    robot->addSensorInterpTask("ROSPublishingTask", 100, &myPublishCB);

    RCLCPP_INFO(this->get_logger(), "PhiROS2Aria: Setup complete.");

    return true;
}

ArPose nextPos;

void PhiROS2Aria::publish()
{
    bool okToPublish = false;

    /// Read everything important

    assert(laser);

    std::list<ArSensorReading *>::const_iterator it;
    ArPose poseTaken;

    laser->lockDevice();
    // get laser scan data
    const std::list<ArSensorReading *> *readings = laser->getRawReadings();
    assert(readings);
    RCLCPP_INFO(this->get_logger(), "laserscan:  %lu readings\n", readings->size());

    // get pose when the central reading was taken
    it = readings->begin();
    if(!readings->empty()){
        for(int i=0;i<readings->size()/2;i++)
            ++it;
        poseTaken = (*it)->getPoseTaken();
    }

    // get laser point cloud data
    assert(laser->getCurrentBuffer());
    const std::list<ArPoseWithTime *> *p = laser->getCurrentRangeBuffer()->getBuffer();

    assert(p);
    laser->unlockDevice();

    // Note, this is called via SensorInterpTask callback (myPublishCB, named "ROSPublishingTask"). ArRobot object 'robot' sholud not be locked or unlocked.
    ArPose pos = poseTaken;

    rclcpp::Time now = this->now();
    //RCLCPP_INFO(this->get_logger(), "DEPOIS pose x: %f, pose y: %f, pose angle: %f",pos.getX(),pos.getY(),pos.getTh());
    
    // Convert pose

    nav_msgs::msg::Odometry position;
    position.header.stamp = now;
    position.header.frame_id = frame_id_odom;
    position.child_frame_id = frame_id_base_link;
    position.pose.pose.position.x = pos.getX() / 1000; // Aria returns pose in mm.
    position.pose.pose.position.y = pos.getY() / 1000;
    position.pose.pose.position.z = 0;
    tf2::Transform tf_transform;
    tf2::Quaternion q;
    q.setRPY(0, 0, pos.getTh() * M_PI / 180);
    tf_transform.setRotation(q);
    position.pose.pose.orientation = tf2::toMsg(tf_transform).rotation;
    position.twist.twist.linear.x = robot->getVel() / 1000.0; // Aria returns velocity in mm/s.
    position.twist.twist.linear.y = robot->getLatVel() / 1000.0;
    position.twist.twist.angular.z = robot->getRotVel() * M_PI / 180;

    //pose_pub->publish(position);
    // RCLCPP_INFO(this->get_logger(), "publish: (time %d) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f",
    //             position.header.stamp.sec,
    //             (double)position.pose.pose.position.x,
    //             (double)position.pose.pose.position.y,
    //             (double)position.pose.pose.orientation.w,
    //             (double)position.twist.twist.linear.x,
    //             (double)position.twist.twist.linear.y,
    //             (double)position.twist.twist.angular.z);

    // publishing transform odom->base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = frame_id_odom;
    t.child_frame_id = frame_id_base_link;
    t.transform = tf2::toMsg(tf_transform);
    t.transform.translation.x = position.pose.pose.position.x;
    t.transform.translation.y = position.pose.pose.position.y;
    t.transform.translation.z = position.pose.pose.position.z;

    /*if (isSimulation_)
    {
        Pose simAbsoluteTruePose = getAbsoluteTruePose();
        position.pose.pose.position.x = simAbsoluteTruePose.x;
        position.pose.pose.position.y = simAbsoluteTruePose.y;
        position.pose.pose.position.z = 0;
        q.setRPY(0, 0, simAbsoluteTruePose.theta * M_PI / 180);
        tf_transform.setRotation(q);
        position.pose.pose.orientation = tf2::toMsg(tf_transform).rotation;
        //absTruePose_pub->publish(position);
        
        Pose simRelativeTruePose = getRelativeTruePose();
        position.pose.pose.position.x = simRelativeTruePose.x;
        position.pose.pose.position.y = simRelativeTruePose.y;
        position.pose.pose.position.z = 0;
        q.setRPY(0, 0, simRelativeTruePose.theta * M_PI / 180);
        tf_transform.setRotation(q);
        position.pose.pose.orientation = tf2::toMsg(tf_transform).rotation;
        //relTruePose_pub->publish(position);
    }*/

    // Publish sonar information, if enabled.
    sensor_msgs::msg::PointCloud2 cloud;        // sonar readings.
    std::stringstream sonar_debug_info; // Log debugging info
    if (publish_sonar)
    {
        cloud.header.stamp = position.header.stamp; // copy time.
        // sonar sensors relative to base_link
        cloud.header.frame_id = frame_id_sonar;

        sonar_debug_info << "Sonar readings: ";

        std::vector<geometry_msgs::msg::Point32> points;
        for (int i = 0; i < robot->getNumSonar(); i++)
        {
            ArSensorReading *reading = NULL;
            reading = robot->getSonarReading(i);
            if (!reading)
            {
                RCLCPP_INFO(this->get_logger(), "Did not receive a sonar reading");
                continue;
            }
            // getRange() will return an integer between 0 and 5000 (5m)
            sonar_debug_info << reading->getRange() << " ";

            // add sonar readings (robot-local coordinate frame) to cloud
            geometry_msgs::msg::Point32 p;
            p.x = reading->getLocalX() / 1000.0;
            p.y = reading->getLocalY() / 1000.0;
            p.z = 0.0;
            points.push_back(p);
        }
        //RCLCPP_INFO(this->get_logger(), sonar_debug_info.str().c_str());

        cloud.width = points.size();
        cloud.height = 1;
        cloud.fields.resize(3);

        // Convert x/y/z to fields
        cloud.fields[0].name = "x";
        cloud.fields[1].name = "y";
        cloud.fields[2].name = "z";

        // All offsets are *4, as all field data types are float32
        int offset = 0;
        for (size_t d = 0; d < cloud.fields.size(); ++d, offset += 4)
        {
            cloud.fields[d].offset = offset;
            cloud.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
            cloud.fields[d].count = 1;
        }
        cloud.point_step = offset;
        cloud.row_step = cloud.point_step * cloud.width;

        cloud.data.resize(points.size() * cloud.point_step);
        cloud.is_bigendian = false; // @todo ?
        cloud.is_dense = false;

        // Copy the data points
        for (size_t cp = 0; cp < points.size(); ++cp)
        {
            memcpy(&cloud.data[cp * cloud.point_step + cloud.fields[0].offset], &points[cp].x, sizeof(float));
            memcpy(&cloud.data[cp * cloud.point_step + cloud.fields[1].offset], &points[cp].y, sizeof(float));
            memcpy(&cloud.data[cp * cloud.point_step + cloud.fields[2].offset], &points[cp].z, sizeof(float));
        }

        // publish topic
        //sonar_pub->publish(cloud);
    } // end if sonar_enabled

    // Publish laser information, if enabled.
    sensor_msgs::msg::PointCloud2 laserCloud;        
    if (publish_aria_lasers)
    {
        /*
        assert(laser);

        laser->lockDevice();
        // get laser scan data
        const std::list<ArSensorReading *> *readings = laser->getRawReadings();
        assert(readings);
        RCLCPP_INFO(this->get_logger(), "laserscan:  %lu readings\n", readings->size());

        // get laser point cloud data
        assert(laser->getCurrentBuffer());
        const std::list<ArPoseWithTime *> *p = laser->getCurrentRangeBuffer()->getBuffer();
        assert(p);
        laser->unlockDevice();
        */

        // Publish laserscan
        if (readings->size() > 0)
        {
            laserScanMsg_.header.stamp = now;
            laserScanMsg_.ranges.resize(readings->size());
            size_t n = 0;
            if (laser->getFlipped())
            {
                // Reverse the data
                for (std::list<ArSensorReading *>::const_reverse_iterator r = readings->rbegin(); r != readings->rend(); ++r)
                {
                    assert(*r);
                    if ((*r)->getIgnoreThisReading())
                        laserScanMsg_.ranges[n] = -1;
                    else
                        laserScanMsg_.ranges[n] = (*r)->getRange() / 1000.0;
                    ++n;
                }
            }
            else
            {
                for (std::list<ArSensorReading *>::const_iterator r = readings->begin(); r != readings->end(); ++r)
                {
                    assert(*r);
                    if ((*r)->getIgnoreThisReading())
                        laserScanMsg_.ranges[n] = -1;
                    else
                        laserScanMsg_.ranges[n] = (*r)->getRange() / 1000.0;
                    ++n;
                }
            }
            //laserScan_pub->publish(laserScanMsg_);
        }

        // Publish laser pointcloud
        if (p->size() > 0)
        {
            okToPublish = true;

            laserCloud.header.stamp = position.header.stamp; // copy time.
            laserCloud.header.frame_id = frame_id_laser;

            laserCloud.width = p->size();
            laserCloud.height = 1;
            laserCloud.fields.resize(3);

            // Convert x/y/z to fields
            laserCloud.fields[0].name = "x";
            laserCloud.fields[1].name = "y";
            laserCloud.fields[2].name = "z";

            // All offsets are *4, as all field data types are float32
            int offset = 0;
            for (size_t d = 0; d < laserCloud.fields.size(); ++d, offset += 4)
            {
                laserCloud.fields[d].offset = offset;
                laserCloud.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
                laserCloud.fields[d].count = 1;
            }
            laserCloud.point_step = offset;
            laserCloud.row_step = laserCloud.point_step * laserCloud.width;

            laserCloud.data.resize(p->size() * laserCloud.point_step);
            laserCloud.is_bigendian = false; // @todo ?
            laserCloud.is_dense = false;

            // Copy the data points
            size_t n = 0;
            for (std::list<ArPoseWithTime *>::const_iterator i = p->begin(); i != p->end(); ++i)
            {
                assert(*i);
                float x = (*i)->getX() / 1000.0;
                float y = (*i)->getY() / 1000.0;
                float z = 0.0;
                memcpy(&laserCloud.data[n * laserCloud.point_step + laserCloud.fields[0].offset], &x, sizeof(float));
                memcpy(&laserCloud.data[n * laserCloud.point_step + laserCloud.fields[1].offset], &y, sizeof(float));
                memcpy(&laserCloud.data[n * laserCloud.point_step + laserCloud.fields[2].offset], &z, sizeof(float));
                ++n;
            }
            //laserCloud_pub->publish(laserCloud);
        }

    } // end if laser_enabled

    if(okToPublish)
    {
        pose_pub->publish(position);
        RCLCPP_INFO(this->get_logger(), "publish: (time %d) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f",
                position.header.stamp.sec,
                (double)position.pose.pose.position.x,
                (double)position.pose.pose.position.y,
                (double)position.pose.pose.orientation.w,
                (double)position.twist.twist.linear.x,
                (double)position.twist.twist.linear.y,
                (double)position.twist.twist.angular.z);

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        // absTruePose_pub->publish(position);
        // relTruePose_pub->publish(position);

        sonar_pub->publish(cloud);
        RCLCPP_INFO(this->get_logger(), sonar_debug_info.str().c_str());

        laserScan_pub->publish(laserScanMsg_);
        laserCloud_pub->publish(laserCloud);
    }

    // stop robot if no cmd_vel message was received for 0.6 seconds
    robot->lock();
    if (now - veltime > rclcpp::Duration(0, 600000000))
    {
        robot->setVel(0.0);
        if (robot->hasLatVel())
            robot->setLatVel(0.0);
        robot->setRotVel(0.0);
    }
    robot->unlock();
}

void PhiROS2Aria::cmdvel_cb(const geometry_msgs::msg::Twist::ConstSharedPtr &msg)
{
    robot->lock();

    veltime = this->now();
    RCLCPP_INFO(this->get_logger(), "cmd_vel:  new speed: [%0.2f,%0.2f]", msg->linear.x * 1e3, msg->angular.z);

    robot->setVel(msg->linear.x * 1e3);
    if (robot->hasLatVel())
        robot->setLatVel(msg->linear.y * 1e3);
    robot->setRotVel(msg->angular.z * 180 / M_PI);

    RCLCPP_INFO(this->get_logger(), "sent vels to Aria: x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s",
                (double)msg->linear.x * 1e3, (double)msg->linear.y * 1.3, (double)msg->angular.z * 180 / M_PI);

    robot->unlock();
}

// GLOBAL VARIABLES FOR THE SIMULATOR
double theX = 0.0;
double theY = 0.0;
double theTh = 0.0;
double initialX = 0.0;
double initialY = 0.0;
double initialTh = 0.0;
double sinInitTh = 0.0;
double cosInitTh = 1.0;
int firstPass = true;
std::string fileName("");

bool PhiROS2Aria::simStatPacketHandler(ArRobotPacket *packet)
{

    // if packet cannot be handled return false, else return true
    switch (packet->getID())
    {
    case 0x62:
    {
        // Empty bytes
        char a = packet->bufToByte(); // unused byte
        char b = packet->bufToByte(); // unused byte
        ArTypes::UByte4 flags = packet->bufToUByte4();

        // Get simulation clock intervals (nominal, measured, last)
        int simint = packet->bufToUByte2();
        int realint = packet->bufToUByte2();
        int lastint = packet->bufToUByte2();

        // Get true pose
        int iTheX = packet->bufToByte4();
        int iTheY = packet->bufToByte4();
        int realZ = packet->bufToByte4();
        int iTheTh = packet->bufToByte4();
        //    cout << iTheTh << ' ';

        theX = iTheX / 1000.0;
        theY = iTheY / 1000.0;
        theTh = iTheTh; //*M_PI/180.0;

        if (firstPass)
        {
            initialX = theX;
            initialY = theY;
            initialTh = theTh;

            cosInitTh = cos(DEG2RAD(-initialTh));
            sinInitTh = sin(DEG2RAD(-initialTh));

            firstPass = false;
        }

        // Get geopositioning data
        if (flags & ArUtil::BIT1)
        {
            double lat = packet->bufToByte4() / 10e6;
            double lon = packet->bufToByte4() / 10e6;
            double alt = packet->bufToByte4() / 100;
            printf("\tLatitude = %f deg., Longitude = %f deg., Altitude = %f m\n", lat, lon, alt);
        }
        return true;
    }
    case 0x66:
    { // SIM_MAP_CHANGED packet
        // Did I opened the file?
        unsigned char user = packet->bufToUByte(); // unused byte
        // Was it loaded or just reopened without changes (a.k.a. selected the same map)
        unsigned char loaded = packet->bufToUByte(); // unused byte

        // get fileName
        int dataLength = packet->getDataLength();
        int remainingData = dataLength - packet->getReadLength();
        fileName.resize(remainingData);
        packet->bufToStr(&fileName[0], remainingData);

        std::cout << "\n\n\n\nFileName: " << fileName << "\n\n\n\n";

        return true;
    }
    default:
        return false;
    }
}

Pose PhiROS2Aria::getAbsoluteTruePose()
{
    Pose simAbsoluteTruePose;
    simAbsoluteTruePose.x = theX;
    simAbsoluteTruePose.y = theY;
    simAbsoluteTruePose.theta = theTh;

    while (simAbsoluteTruePose.theta > 180.0)
        simAbsoluteTruePose.theta -= 360.0;
    while (simAbsoluteTruePose.theta < -180.0)
        simAbsoluteTruePose.theta += 360.0;

    return simAbsoluteTruePose;
}

Pose PhiROS2Aria::getRelativeTruePose()
{
    Pose simRelativeTruePose;
    simRelativeTruePose.x = cosInitTh * (theX - initialX) - sinInitTh * (theY - initialY);
    simRelativeTruePose.y = sinInitTh * (theX - initialX) + cosInitTh * (theY - initialY);
    simRelativeTruePose.theta = theTh - initialTh;

    while (simRelativeTruePose.theta > 180.0)
        simRelativeTruePose.theta -= 360.0;
    while (simRelativeTruePose.theta < -180.0)
        simRelativeTruePose.theta += 360.0;

    return simRelativeTruePose;
}
