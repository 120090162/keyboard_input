#include "../include/keyboard_input/keyboard_input.h"

KeyboardInput::KeyboardInput(ros::NodeHandle &_nh)
{
    nh = _nh;
    publisher_ = nh.advertise<sensor_msgs::Joy>("/joy", 10);
    timer_ = nh.createTimer(ros::Duration(0.0001), std::bind(&KeyboardInput::timer_callback, this));
    inputs_ = sensor_msgs::Joy();
    inputs_.axes = std::vector<float>(8, 0.0);
    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
    ROS_INFO("Node initialized. Please input keys, press Ctrl+C to quit.");
}

void KeyboardInput::timer_callback()
{
    if (kbhit())
    {
        char key = getchar();
        check_command(key);
        if (inputs_.buttons[0] == 0 || inputs_.buttons[4] == 0)
            check_value(key);
        else
        {
            inputs_.axes = std::vector<float>(8, 0.0);
            reset_count_ = 100;
        }
        publisher_.publish(inputs_);
        just_published_ = true;
    }
    else
    {
        if (just_published_)
        {
            reset_count_ -= 1;
            if (reset_count_ == 0)
            {
                just_published_ = false;
                if (inputs_.buttons[0] != 0 && inputs_.buttons[4] != 0)
                {
                    inputs_.buttons[0] = 0;
                    inputs_.buttons[4] = 0;
                    publisher_.publish(inputs_);
                }
            }
        }
    }
}

void KeyboardInput::check_command(const char key)
{
    inputs_.buttons = std::vector<int>(11, 0);
    switch (key)
    {
    case 'f':
    case 'F':
        inputs_.buttons[0] = 1; // A
        break;
    case 'q':
    case 'Q':
        inputs_.buttons[6] = 1; // lb
        break;
    default:
        break;
    }
}

void KeyboardInput::check_value(char key)
{
    switch (key)
    {
    case 'w':
    case 'W':
        inputs_.axes[1] = min<float>(inputs_.axes[1] + sensitivity_left_, 1.0);
        break;
    case 's':
    case 'S':
        inputs_.axes[1] = max<float>(inputs_.axes[1] - sensitivity_left_, -1.0);
        break;
    case 'd':
    case 'D':
        inputs_.axes[2] = min<float>(inputs_.axes[2] + sensitivity_left_, 1.0);
        break;
    case 'a':
    case 'A':
        inputs_.axes[2] = max<float>(inputs_.axes[2] - sensitivity_left_, -1.0);
        break;

    case 'i':
    case 'I':
        inputs_.axes[7] = min<float>(inputs_.axes[7] + sensitivity_right_, 1.0);
        break;
    case 'k':
    case 'K':
        inputs_.axes[7] = max<float>(inputs_.axes[7] - sensitivity_right_, -1.0);
        break;
    case 'l':
    case 'L':
        inputs_.axes[0] = min<float>(inputs_.axes[0] + sensitivity_right_, 1.0);
        break;
    case 'j':
    case 'J':
        inputs_.axes[0] = max<float>(inputs_.axes[0] - sensitivity_right_, -1.0);
        break;
    case 'u':
    case 'U':
        inputs_.axes[6] = min<float>(inputs_.axes[6] + sensitivity_right_, 1.0);
        break;
    case 'o':
    case 'O':
        inputs_.axes[6] = max<float>(inputs_.axes[6] - sensitivity_right_, -1.0);
        break;
    default:
        break;
    }
}

bool KeyboardInput::kbhit()
{
    timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_input_node");
    ros::NodeHandle nh;
    std::unique_ptr<KeyboardInput> a1 = std::make_unique<KeyboardInput>(nh);

    ros::spin();
    ros::shutdown();
    return 0;
}
