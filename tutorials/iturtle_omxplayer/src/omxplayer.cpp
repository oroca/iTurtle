#include <signal.h> // kill
#include <sys/types.h>
#include <sys/wait.h> // waitpid
#include <unistd.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>


#define PROJECT_NAME CATKIN_PROJECT_NAME
#define SHARED_FOLDER CATKIN_PACKAGE_SHARE_DESTINATION

static const char SEP = '/';
static const std::string g_package_name = PROJECT_NAME;
static const std::string g_share_directory = SHARED_FOLDER;

static std::string g_current_directory = ".";
static std::string g_package_directory = ".";


pid_t g_play_pid = -1;
std::string g_sound_file_path = "";
std::string g_sound_output = "hdmi"; // hdmi/local/both/alsa
ros::Publisher g_done_msg_pub;

static void wait_for_child(int child)
{
  int status;
  while (-1 == waitpid(child, &status, 0))
    ;
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    printf( "Process %d failed\n", child);
  }
}

void play_sound_callback(const std_msgs::String::ConstPtr &msg)
{
  std_msgs::String done_msg;

  if (msg->data == "")
  {
    if (g_play_pid != -1)
      kill(g_play_pid, SIGKILL);

    g_play_pid = -1;
    done_msg.data = "play_sound_fail";
    g_done_msg_pub.publish(done_msg);
    return;
  }

  if (g_play_pid != -1) {
    kill(g_play_pid, SIGKILL);
  }

  g_play_pid = fork();

  switch (g_play_pid)
  {
  case -1:
    fprintf(stderr, "Fork Failed!! \n");
    done_msg.data = "play_sound_fail";
    g_done_msg_pub.publish(done_msg);
    break;
  case 0:
    execl("/usr/bin/omxplayer", "omxplayer", "-o", g_sound_output.c_str(), (g_sound_file_path + msg->data).c_str(), (char*)0);
    //execl("/usr/bin/omxplayer", "omxplayer", (g_sound_file_path + msg->data).c_str(), (char *)0);
    break;
  default:
    wait_for_child(g_play_pid);
    g_play_pid = -1;
    done_msg.data = "play_sound";
    g_done_msg_pub.publish(done_msg);
    break;
  }
}

inline bool exists (const std::string& name) 
{
    return access( name.c_str(), F_OK ) != -1;
}

inline std::string dirname(const std::string& path) 
{
  size_t last = path.find_last_of(SEP);
  if (last == std::string::npos) 
  {
    return path;
  }
  return path.substr(0, last);
}

int main(int argc, char **argv)
{
  g_current_directory = dirname(argv[0]) + SEP;
  ros::package::getPath("iturtle_omxplayer");

  ros::init(argc, argv, "omxplayer");
  ros::NodeHandle nh("~");

  g_sound_file_path = nh.param<std::string>("sound_file_path", "");

  if (g_sound_file_path.empty() || !exists(g_sound_file_path)) 
  {
    printf("sound_file_path doesn't exists, will uses current path\n", g_sound_file_path.c_str());
    g_sound_file_path = g_current_directory;
  }

  if (g_sound_file_path.back() != SEP) 
  {
    g_sound_file_path += SEP;
  }
  printf("param: sound_file_path:%s\n", g_sound_file_path.c_str());

  // private param
  g_sound_output = nh.param<std::string>("sound_output", "hdmi");
  printf("param: sound_output=%s\n", g_sound_output.c_str());

  ros::Subscriber play_mp3_sub = nh.subscribe("/play_sound_file", 10, &play_sound_callback);
  g_done_msg_pub = nh.advertise<std_msgs::String>("/omxplayer/play_done", 5);

  ros::spin();
  return 0;
}
