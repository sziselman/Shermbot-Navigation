#include "rigid2d/diff_drive.hpp"
#include <iostream>
#include <cmath>
#include <string>

namespace rigid2d
{
    std::ostream & operator<<(std::ostream & os, const DiffDrive & dd)
    {
        os << '(' << dd.x << ', ' << dd.y << ', ' << dd.th << ')' << std::endl;
    }

    std::istream & operator<<(std::istream & is, DiffDrive & dd)
    {
        is >> dd.wheelBase;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> dd.wheelBase;
        }

        is >> dd.wheelRad;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> dd.wheelRad;
        }

        is >> dd.x;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> dd.x;
        }

        is >> dd.y;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> dd.y;
        }

        is >> dd.th;
        while (is.fail()) {
            is.clear();
            is.ignore();
            is >> dd.th;
        }
        
        return is;
    }
}