#include "DriveTrain.h"
#include <unistd.h>
#include <utility> // pair
#include <math.h>

class RoomeTest {
public:
    ArduinoSerial get_serial() {
        return d_train.serial;
    }

private:
    DriveTrain d_train;

};

std::pair<int, int> get_odm(const std::string& odm_str) {
    auto no_markers_str = odm_str.substr(1, odm_str.find('>') - 1);
    int l_odm = stoi(no_markers_str.substr(0, no_markers_str.find(',')));
    int r_odm = stoi(no_markers_str.substr(no_markers_str.find(',')+1));
    return std::make_pair(l_odm, r_odm);
}

//Arduino follows format <State,forwards/backwards,left/right,emergencystop,distance,angle,piconfirmation>
//State can be 0 for standby, 1 for turn, and 2 for drive straight
bool drive_go_straight(RoomeTest& test_c) {
    auto serial = test_c.get_serial();
    serial.write_string("<2,0,0,0,0.1,0,1>");
    auto odm_str = serial.read_string();
    auto odm_p = get_odm(odm_str);
    bool pass = std::abs(odm_p.first - 6) < 2 && std::abs(odm_p.second - 6) < 2;
    if (!pass) {
        std::cout << "Straight ODM: " << odm_str << std::endl;
    }
    return pass;
}

bool drive_turn_right(RoomeTest& test_c) {
    auto serial = test_c.get_serial();
    serial.write_string("<1,0,1,0,0,90,1>");
    auto odm_str = serial.read_string();
    auto odm_p = get_odm(odm_str);
    bool pass = std::abs(odm_p.first - 20) < 2 && odm_p.second == 0;
    if (!pass) {
        std::cout << "Right turn ODM: " << odm_str << std::endl;
    }
    return pass;
}

bool drive_turn_left(RoomeTest& test_c) {
    auto serial = test_c.get_serial();
    serial.write_string("<1,0,0,0,0,90,1>");
    auto odm_str = serial.read_string();
    auto odm_p = get_odm(odm_str);
    bool pass = odm_p.first == 0 && std::abs(odm_p.second - 20) < 2;
    if (!pass) {
        std::cout << "Left turn ODM: " << odm_str << std::endl;
    }
    return pass;
}

int main() {
    RoomeTest test_c;
    std::cout << "Test Straight: ";
    if (drive_go_straight(test_c)) {
        std::cout << "Pass!" << std::endl;
    } else {
        std::cout << "Fail!" << std::endl;
    }
    sleep(2);
    std::cout << "Test Right: ";
    if (drive_turn_right(test_c)) {
        std::cout << "Pass!" << std::endl;
    } else {
        std::cout << "Fail!" << std::endl;
    }

    sleep(2);
    std::cout << "Test Left: ";
    if (drive_turn_left(test_c)) {
        std::cout << "Pass!" << std::endl;
    } else {
        std::cout << "Fail!" << std::endl;
    }
    return 0;
}
