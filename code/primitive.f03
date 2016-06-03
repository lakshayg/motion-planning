! program to calculate path between 2 configurations
! of a 2-link robot arm. Initial and final positions
! of the end effector are taken as the input and all
! the intermediate positions are calculated using IK
program primitive
    implicit none
    real, parameter :: PI = 4.*atan(1.)
    real, parameter :: DEG = 180./PI
    real, parameter :: L1 = 10., L2 = 7. ! link lengths in cm
    real, parameter :: t_max = 1         ! simulation time [sec]
    integer, parameter :: N = 100        ! steps in simulation
    real, parameter :: dt = t_max/(N-1)
    real :: x0, y0, x1, y1               ! initial and final pos
    real :: a0, b0, a1, b1               ! initial, final angles
    real, dimension(1,N) :: a, b, x, y
    real, dimension(1,4) :: c0, c1       ! best fit cubics
    real, dimension(N) :: t              ! time
    real, dimension(4,N) :: v            ! [1 t t^2 t^3]'
    real, dimension(1,N) :: adot, bdot 
    integer :: i

    ! take user input
    100 format(A)
    write (*,100,advance="no") 'Start position: '
    read (*,*) x0, y0
    write (*,100,advance="no") 'End position:   '
    read (*,*) x1, y1

    ! convert from position to joint space
    call ik_solver(x0, y0, a0, b0)
    call ik_solver(x1, y1, a1, b1)

    ! fit a cubic function between configurations
    c0(:,1:4) = cubic(a0, a1, 0., 0.)
    c1(:,1:4) = cubic(b0, b1, 0., 0.)

    t = [(i*t_max/(N-1),i=0,N-1)]
    v = vandermonde(t,4)
    a = matmul(c0, v)
    b = matmul(c1, v)
    adot(1,2:N) = (a(1,2:N) - a(1,1:N-1))/dt
    bdot(1,2:N) = (b(1,2:N) - b(1,1:N-1))/dt
    x = L1 * cos(a) + L2 * cos(b)
    y = L1 * sin(a) + L2 * sin(b)

    print *, '# ----------------------------------------------------- #'
    print *, '# T       alpha   beta    adot    bdot    x       y     #'
    print *, '# ----------------------------------------------------- #'
    do i=1,N
        print "(7f8.3)", t(i), a(1,i), b(1,i), adot(1,i), bdot(1,i), x(1,i), y(1,i)
    end do

contains

    function vandermonde(t, n)
        ! construct the vandermonde matrix from a given list
        ! [ 1         1   1   1   ...   1 ]
        ! [ t1        t2  t2  t3  ...   tk]
        ! [ :         :   :   :         : ]
        ! [ t1^(n-1)  ...                 ]
        real, dimension(:), intent(in) :: t
        integer, intent(in) :: n
        real, dimension(n,size(t)) :: vandermonde
        integer :: i
        vandermonde(1,:) = [(1.,i=1,size(t))]
        do i=2,n
            vandermonde(i,:) =  t * vandermonde(i-1,:)
        end do
    end function vandermonde

    function angle(a,b,c)
        ! calculate the angle opposite to side
        ! 'a' of a triangle using cosine law
        real, intent(in) :: a, b, c
        real :: angle
        angle = acos((b*b + c*c - a*a)/(2*b*c))
    end function angle

    subroutine ik_solver(x, y, a, b)
        ! calculate the arm joint angles given
        ! the (x,y) coordinate of end effector
        real, intent(in) :: x, y
        real, intent(out) :: a, b
        real :: L
        L = sqrt(x*x + y*y)
        a = angle(L2, L1, L) + atan(y/x)
        b = angle(L, L1, L2) - (PI - a)
    end subroutine ik_solver

    function cubic(x0, x1, xd0, xd1)
        ! fitting a cubic x(t): [0,1] -> R
        ! using the conditions on initial
        ! and final values and derivatives
        real, intent(in) :: x0, x1, xd0, xd1
        real, dimension(1,4) :: cubic
        real, dimension(4,4) :: A
        real, dimension(4,1) :: b
        A = reshape([1, 0, -3,  2,  &
                     0, 0,  3, -2,  &
                     0, 1, -2,  1,  &
                     0, 0, -1,  1], [4,4])
        b = reshape([x0, x1, xd0, xd1], [4,1])
        cubic = transpose(matmul(A,b))
    end function cubic

end program primitive
