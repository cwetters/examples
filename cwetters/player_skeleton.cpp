// Author(s):         Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "ai_base.hpp"

#include <boost/lexical_cast.hpp>

#include <fstream>
#include <iostream>

class my_ai
  : public aiwc::ai_base
{
  static constexpr double PI = 3.1415926535;

public:
  my_ai(std::string server_ip, std::size_t port, std::string realm, std::string key, std::string datapath)
    : aiwc::ai_base(std::move(server_ip), port, std::move(realm), std::move(key), std::move(datapath))
  {
    // you don't have any information of the game here
  }

private:
  // this function is called at the beginning of a game
  void init()
  {
    // from here, you have access to game-specific constant information such as field dimensions
    // check example 'general_check-variables_cpp' to see what information are available

    // you can initialize some customvariables here
    
    goal_posts[0][0] = 3.9;
    goal_posts[0][1] = 0.5;
    goal_posts[1][0] = 3.9;
    goal_posts[1][1] = 0.0;
    goal_posts[2][0] = 3.9;
    goal_posts[2][1] = -0.5;
    own_goal_posts[0][0] = -3.9;
    own_goal_posts[0][1] = 0.5;
    own_goal_posts[1][0] = -3.9;
    own_goal_posts[1][1] = 0.0;
    own_goal_posts[2][0] = -3.9;
    own_goal_posts[2][1] = -0.5;
    goal_rel[3][0] = -0.3;
    goal_rel[3][1] = 0.0;
    goal_rel[4][0] = -0.3;
    goal_rel[4][1] = 0.0;
    goal_rel[1][0] = 0.7;
    goal_rel[1][1] = 0.0;
    goal_rel[2][0] = 0.7;
    goal_rel[2][1] = 0.0;
    goal_rel[0][0] = 0.9;
    goal_rel[0][1] = 0.0;
      }

  ///********* from player_rulebased-A.cpp
  auto get_coord(const aiwc::frame& f)
  {
    //Get data in the frame
    decltype(cur_posture) cur;
    decltype(cur_posture_op) cur_op;
      std::array<double, 2> pos_ball = { (*f.opt_coordinates).ball.x, (*f.opt_coordinates).ball.y };

    for(std::size_t id = 0; id < 5; ++id) {
      cur[id][X]  = (*f.opt_coordinates).robots[MYTEAM][id].x;
      cur[id][Y]  = (*f.opt_coordinates).robots[MYTEAM][id].y;
      cur[id][TH] = (*f.opt_coordinates).robots[MYTEAM][id].th;
      cur[id][ACTIVE] = (*f.opt_coordinates).robots[MYTEAM][id].active;
      cur[id][TOUCH] = (*f.opt_coordinates).robots[MYTEAM][id].touch;

      cur_op[id][X]  = (*f.opt_coordinates).robots[OPPONENT][id].x;
      cur_op[id][Y]  = (*f.opt_coordinates).robots[OPPONENT][id].y;
      cur_op[id][TH] = (*f.opt_coordinates).robots[OPPONENT][id].th;
      cur_op[id][ACTIVE] = (*f.opt_coordinates).robots[MYTEAM][id].active;
      cur_op[id][TOUCH] = (*f.opt_coordinates).robots[MYTEAM][id].touch;
    }

    return std::make_tuple(cur, cur_op, pos_ball);
  }
  ///******


  // this function is called at each timestep. in 'f', current step's information are stored
  // check example 'general_check-variables_cpp' to see what information are available here
  // you should implement an AI soccer algorithm that sets robot wheel velocities for each timestep here
  void update(const aiwc::frame& f)
  {
    if(f.reset_reason == aiwc::GAME_START) {
      std::cout << "Game started : " << f.time << std::endl;
    }
    if(f.reset_reason == aiwc::SCORE_MYTEAM) {
      // yay! my team scored!
      std::cout << "Myteam scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::SCORE_OPPONENT) {
      // T_T what have you done
      std::cout << "Opponent scored : " << f.time << std::endl;
    }
    else if(f.reset_reason == aiwc::GAME_END) {
      // game is finished. finish() will be called after you return.
      // now you have about 30 seconds before this process is killed.
      std::cout << "Game ended : " << f.time << std::endl;
      return;
    }

    std::tie(cur_posture, cur_posture_op, cur_ball) = get_coord(f);

    if(f.opt_coordinates) { // if the optional coordinates are given,
      // const auto& myteam   = f.opt_coordinates->robots[MYTEAM];
      // const auto& opponent = f.opt_coordinates->robots[OPPONENT];
      // const auto& ball     = f.opt_coordinates->ball;
      //
      // const auto& myteam0_x      = (*f.opt_coordinates).robots[MYTEAM][0].x;
      // const auto& myteam0_y      = (*f.opt_coordinates).robots[MYTEAM][0].y;
      // const auto& myteam0_th     = (*f.opt_coordinates).robots[MYTEAM][0].th;
      // const auto& myteam0_active = (*f.opt_coordinates).robots[MYTEAM][0].active;
      // const auto& myteam0_touch  = (*f.opt_coordinates).robots[MYTEAM][0].touch;
      //
      // for (int i = 0; i < 5; i++) {
      //   if ((*f.opt_coordinates).robots[MYTEAM][i].touch) {
      //     std::cout << "My robot " << i << " touched the ball" << std::endl;
      //   }
      // }
    }
    else { // given no coordinates, you need to get coordinates from image
    }

    //std::array<double, 10> wheels;
    //for(int i = 0; i < 2*info.number_of_robots; i++)
      //wheels[i] = info.max_linear_velocity[i/2];
    count = count + 1;
    if (count > refreshAfter){
      count = 0;
      goalPosUpdate();
    }
    //std::cout << cur_posture[0][0] << std::endl;
    //std::cout << cur_posture[0][1] << std::endl;
    //std::cout << cur_posture[0][2] << std::endl;
    //std::cout << goal_pos[0][0] << std::endl;
    //std::cout << goal_pos[0][1] << std::endl;
    //std::cout << goal_pos[0][2] << std::endl;
    for(int i=0; i<5; i++)
    {
      drivePos(i);
    }

    set_wheel(wheels); 
  }

  double d2r(double deg) // convert degrees to radians 
  {
    return deg * PI / 180;
  }

  double r2d(double rad) // convert radians to degrees (mostly for readout) 
  {
    return rad * 180 / PI;
  }

  void wrapHeadings(){ //wrap headings between -PI and PI
    for(int i=0; i<5; i++)
    {
      while(cur_posture[i][2] > PI)
      {
        cur_posture[i][2] = cur_posture[i][2] - 2*PI;
      }
      while(cur_posture[i][2] < -PI)
      {
        cur_posture[i][2] = cur_posture[i][2] + 2*PI;
      }
    }
  }

  void goalPosUpdate() //update goal position to be in a scoop formation around current ball location pointed goal-wards
  {
    goal_dir[4] = std::atan2(goal_posts[0][1] - 0, goal_posts[0][0] - 0);
    goal_dir[3] = std::atan2( goal_posts[2][1] - 0, goal_posts[2][0] - 0);
    goal_dir[1] = std::atan2(own_goal_posts[0][1] - 0, own_goal_posts[0][0] - 0);
    goal_dir[0] = std::atan2(own_goal_posts[1][1] - 0, own_goal_posts[1][0] - 0);
    goal_dir[2] = std::atan2(own_goal_posts[2][1] - 0, own_goal_posts[2][0] - 0);

    for (int i = 0; i < 5; i++) // set relative positions rotated around point p by direction theta
    {
      double ctheta = std::cos(goal_dir[i]);
      double stheta = std::sin(goal_dir[i]);
      goal_pos[i][0] = cur_ball[0] + goal_rel[i][0]*ctheta - goal_rel[i][1]*stheta;
      goal_pos[i][1] = cur_ball[1] + goal_rel[i][1]*ctheta + goal_rel[i][0]*stheta;
      goal_pos[i][2] = goal_dir[i];
      if (i > 0)
      {
        goal_pos[i][0] = std::min(2.9, goal_pos[i][0]);
      }

      goal_pos[i][0] = std::max(-3.8, goal_pos[i][0]);
      //goal_pos[i][0] = goal_rel[i][0];
      //goal_pos[i][1] = goal_rel[i][1];
      //goal_pos[i][2] = cur_posture[i][2];
    }

  }
  double wrapHeadingDiff(double h) // wrap [0, 2*PI]
    {
     while(h < 0)
    {
      h = h + 2*PI;
    }
    while(h > 2*PI)
    {
      h = h - 2*PI;
    }
    return h;
  }

  double MinHeadingDiff(double h, double g) //pos diff is counter-clockwise turn, neg diff is clockwise turn
  {
    //std::cout << "my dir" << r2d(h) << "goal dir" << r2d(g) << std::endl;
    double d = wrapHeadingDiff(g-h); //counter-clockwise turn distance
    //std::cout << "diff" << r2d(d) << std::endl;
    double dd = 2*PI - d; //clockwise turn distance
    if (d > dd)
    {
      return -1*dd; //clockwise is closer, indicate with negative
    } else
    {
      return d; //counter-clockwise is closer
    }  
    
  }

  void drivePos(std::size_t id) // given a target position from goal_pos, set drive wheels to get there
  {
    double to_travel_x = goal_pos[id][0] - cur_posture[id][0];
    double to_travel_y = goal_pos[id][1] - cur_posture[id][1];
    //double to_spin = goal_pos[id][2] - cur_posture[id][2];

    double robot_to_goal = std::atan2(to_travel_y, to_travel_x);
    //td::cout << "robot_to_goal" << r2d(robot_to_goal) << std::endl;

    double robot_diff_f_dir = MinHeadingDiff(wrapHeadingDiff(cur_posture[id][2]), wrapHeadingDiff(robot_to_goal)) ;
    double robot_diff_b_dir = MinHeadingDiff(wrapHeadingDiff(cur_posture[id][2] + PI), wrapHeadingDiff(robot_to_goal));

    bool forward = std::abs(robot_diff_f_dir) < std::abs(robot_diff_b_dir);
    
    //std::cout << r2d(robot_diff_f_dir) << "diff f" << std::endl;
    //std::cout << r2d(robot_diff_b_dir) << "diff b" << std::endl;
    double speed = info.max_linear_velocity[id];

    if (forward){
      if (robot_diff_f_dir > 0) // left forward turning
      {
        wheels[id*2 + 1] = speed; //right wheel full speed
        wheels[id*2] = speed*(std::abs(robot_diff_f_dir)*-2.0/PI + 1); //slow left wheel as you need to turn more
      } else { // right forward turning
        wheels[id*2] = speed; // left wheel full speed
        wheels[id*2+1] = speed*(std::abs(robot_diff_f_dir)*-2.0/PI + 1); //slow right wheel as you need to turn more
      } 
    }else{
      if (robot_diff_b_dir > 0) // back right turning
      {
        wheels[id*2] = -1*speed; //left wheel full back speed
        wheels[id*2+1] = -1*speed*(std::abs(robot_diff_b_dir)*-2.0/PI + 1); // slow right wheel back as you need to turn more

      } else { // back left turning
        wheels[id*2+1] = -1*speed; // right wheel full back speed
        wheels[id*2] = -1*speed*(std::abs(robot_diff_b_dir)*-2.0/PI + 1); // slow left wheel back as you need to turn more
      }

    }
     

  }

  void finish()
  {
    // You have less than 30 seconds before it's killed.
    std::ofstream ofs(datapath + "/result.txt");
    ofs << "my_result" << std::endl;
  }

private: // member variable

std::array<std::array<double, 5>, 5> cur_posture; //X, Y, THETA, ACTIVE, TOUCH
std::array<std::array<double, 5>, 5> cur_posture_op;
std::array<double, 2> cur_ball; // X, Y
double time;
int count = 0; // count of number of frames seen since last refresh
int refreshAfter = 1; // how often to refresh goals

std::array<std::array<double, 3>, 5> goal_pos; //X, Y, THETA
std::array<std::array<double, 2>, 5> goal_rel; //X, Y
std::array<double, 5> goal_dir;

std::array<std::array<double, 2>, 3> goal_posts; // boundary of the goal box
std::array<std::array<double, 2>, 3> own_goal_posts; // boundary of the goal box
std::array<double, 10> wheels;

};

int main(int argc, char *argv[])
{
  if(argc < 6) {
    std::cerr << "Usage " << argv[0] << " server_ip port realm key datapath" << std::endl;
    return -1;
  }

  const auto& server_ip = std::string(argv[1]);
  const auto& port      = boost::lexical_cast<std::size_t>(argv[2]);
  const auto& realm     = std::string(argv[3]);
  const auto& key       = std::string(argv[4]);
  const auto& datapath  = std::string(argv[5]);

  my_ai ai(server_ip, port, realm, key, datapath);

  ai.run();

  return 0;
}
