! this program calculates the neighbourhood of a n input
! point. This demonstrates that the neighborhood in joint
! angle space is not the same as the neighbourhood in
! the x-y space
program neighbors
    implicit none
    real, parameter :: pi = 4.*atan(1.)
    real, parameter :: deg = pi/180.
    real, parameter :: L1 = 10., L2 = 7. ! link lengths
    integer, parameter :: N = 5          ! size of neighborhood
    real, parameter :: dT = 5*deg        ! discretization size
    real :: x0, y0                       ! user input point
    real :: a0, b0                       ! angles at initial position
    real :: a, b, x, y


    write (*,"(A)",advance="no") 'Point: ' 
    read (*,*) x0, y0
    call ik_solver(x0, y0, a0, b0)
    
    a = a0 - N*dT
    do while (a <= a0 + N*dT)
        b = b0 - N*dT
        do while (b <= b0 + N*dT)
            x = L1 * cos(a) + L2 * cos(a+b)
            y = L1 * sin(a) + L2 * sin(a+b)
            print "(2f8.3)", x, y
            b = b + dT
        end do
        a = a + dT
    end do

contains

    subroutine ik_solver(x, y, a, b)
        real, intent(in) :: x, y
        real, intent(out) :: a, b
        real :: L, t0, t1
        L = sqrt(x*x + y*y) 
        t0 = acos((L*L + L1*L1 - L2*L2)/(2.*L*L1))
        t1 = acos((L2*L2 + L1*L1 - L*L)/(2.*L2*L1))
        a = t0 + atan2(y,x)
        b = t1 - pi
    end subroutine ik_solver

end program neighbors
