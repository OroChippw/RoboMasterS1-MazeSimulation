#include <webots/Robot.hpp>

#define TIME_STEP 64
#define PI 3.141592653589793116
#define HALF_PI 1.570796326794896558

class MazeBot {


    public:
        MazeBot(Robot *robot){
            this->robot = robot;
        }

        inline bool step()
        {
            return robot->step(TIME_STEP) != -1;
        }

        ~MazeBot(){
            if (robot != nullptr){
                delete robot;
            }
        }

    private:
        Robot *robot;
        
        const double PS_THRESHOLD = 80;
        const double speed = 20;

};