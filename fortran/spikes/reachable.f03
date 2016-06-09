! all the reachable configurations of the robot arm
program reachable
    implicit none
    real, parameter :: PI = 4. * atan(1.)
    real, parameter :: DEG = PI / 180.
    real, parameter :: L1 = 9., L2 = 7.                ! link lengths [cm]
    real, parameter :: dT = 2e-2                       ! simulation step size [rad]
    real, parameter :: a_min = 15*DEG, a_max = 75*DEG
    real, parameter :: b_min = -45*DEG, b_max = 45*DEG
    real :: a = 0, b = 0                               ! joint angles [rad]
    real :: x, y, h, k                                 ! end effector position

    a = a_min
    do while (a .le. a_max)
        b = b_min
        do while (b .le. b_max)
            h = L1 * cos(a)
            k = L1 * sin(a)
            x = h + L2 * cos(a+b)
            y = k + L2 * sin(a+b)
            b = b + dT
            print "(2f8.3)", x, y
            print "(2f8.3)", h, k
        end do
        a = a + dT
    end do

end program reachable
