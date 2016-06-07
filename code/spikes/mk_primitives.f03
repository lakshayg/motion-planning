! program for generating motion primitives from
! origin to all its neighbors for different
! starting and ending velocities
program mk_primitives
    implicit none
    real, parameter :: pi = 4. * atan(1.)
    integer, parameter :: dof = 2       ! degrees of freedom
    real, parameter :: deg = pi/180.    ! convert deg to radians
    real, parameter :: eps = 1e-6       ! precision
    real, parameter :: step = 1./99.    ! time step used in simulation
    real, parameter :: L(2) = [10., 7.] ! link lengths [cm]
    real, parameter :: dT = 10*deg      ! discretization of state
    integer, parameter :: N = 1         ! defines the neighborhood size
    real, dimension(2,1) :: org = reshape([0.,0.],[2,1])
    real, dimension(2,9) :: vel         ! allowed start/end speeds (9 possible values in 2D)
    real, dimension(dof,1) :: A         ! values of joint angles
    real, dimension(dof,4) :: p         ! best fit cubic
    real, dimension(4,100) :: t
    real, dimension(2,100) :: v
    real, dimension(2,100) :: path
    real, dimension(100) :: x, y
    integer :: n_primitive = 0
    integer :: ii, jj, kk               ! variables for looping
    integer :: i, j, k                  !

    do concurrent(i=1:100,j=1:4)
        t(j,i) = ((i-1)/99.)**(j-1)
    end do

    vel = reshape([-1, -1,  &
                   -1,  0,  &
                   -1,  1,  &
                    0, -1,  &
                    0,  0,  &
                    0,  1,  &
                    1, -1,  &
                    1,  0,  &
                    1,  1], [2,9])

    do concurrent(ii=-N:N,jj=-N:N) ! iterate over all the neighbors
        A(:,1) = [ii, jj] * dT     ! a point in neighborhood
        do concurrent(i=1:9,j=1:9) ! iterate over all start/end vel
            call cubic(org,A,vel(:,i),vel(:,j),p)
            path = matmul(p,t)

            ! compute the joint velocities
            v(:,1) = vel(:,i)
            v(:,2:99) = (path(:,3:100) - path(:,1:98))/(2.*step)
            v(:,100) = vel(:,j)

            if (n_primitive .eq. 4) then
                x = L(1) * cos(path(1,:)) + L(2) * cos(path(2,:)+path(1,:))
                y = L(1) * sin(path(1,:)) + L(2) * sin(path(2,:)+path(1,:))
                do k=1,100
                    print "(2f8.3)", x(k), y(k)
                end do
            end if
            n_primitive = n_primitive + 1
        end do
    end do

    !print "(a,4x,i4)", 'number of primitives:', n_primitive

contains

    pure subroutine cubic(y0, y1, yd0, yd1, p)
        real, intent(in), dimension(2,1) :: y0, y1, yd0, yd1
        real, dimension(2,4), intent(out) :: p
        real :: G(2,4), A(4,4)
        G(:,1) = y0(:,1) ![y0(1), y0(2)]
        G(:,2) = y1(:,1) ![y1(1), y1(2)]
        G(:,3) = yd0(:,1)![yd0(1), yd0(2)]
        G(:,4) = yd1(:,1)![yd1(1), yd1(2)]
        !! TODO: make sure that A matrix is correct
        A = reshape([              &
            1, 0, -3,  2,          &
            0, 0,  3, -2,          &
            0, 1, -2,  1,          &
            0, 0, -1,  1], [4,4])
        p = matmul(G,A)
    end subroutine cubic

    ! inverse kinematics for a 2 link robot arm returns the
    ! joint angles of arm given the position of end effector
    pure subroutine ik_solver(x, y, a, b)
        real, intent(in ) :: x, y
        real, intent(out) :: a, b
        real :: t0, t1, LL
        LL = sqrt(x*x + y*y)
        a = angle(L(2), L(1), LL) + atan2(y,x)
        b = angle(LL, L(1), L(2)) - pi
    end subroutine ik_solver

    ! compute the angle opposite to side 'a' in a triangle
    ! with side lengths equal to a, b and c respectively
    pure function angle(a, b, c)
        real, intent(in) :: a, b, c
        real :: angle
        angle = acos((b*b + c*c - a*a)/(2.*b*c))
    end function angle

end program mk_primitives
