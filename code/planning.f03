module planning
    implicit none
    real, parameter :: pi = 4. * atan(1.)
    real, parameter :: deg = pi/180.

contains

    ! generate a motion primitive from the given conditions
    ! INPUTS:
    !   displacement -- the displacement to be caused by primitive
    !   v_start      -- starting velocity
    !   v_end        -- ending velocity
    !   v_max        -- maximum permitted velocity
    ! OUTPUTS:
    !   poly         -- best fit cubic polynomial for given conditions
    !   time         -- time required for executing this primitive
    subroutine primitive(displacement, v_start, v_end, v_max)
        real, dimension(:), intent(in) :: displacement, v_start, v_end, v_max
        real :: time
        integer :: dof, i
        dof = size(displacement)
        ! it is assumed that the dimensions of arrays are consistent
        ! TODO: write checks for ensuring that this condition is met
        
        ! check if the input is consistent
        if (any(abs(v_start) > v_max) .or. any(abs(v_end) > v_max)) then
            print "(a)", 'Inconsistent inputs: Boundary conditions violate the velocity constraint'
            return
        end if

        ! determine the duration of motion primitive required so that
        ! the arm joints do not exceed their specified maximum omegas
        ! as the trajectory is fit using a cubic function
        ! theta(t) = T0 + T1*t + T2*t^2 + T3*t^3 therefore the velocity
        ! is a quadratic given by: w(t) = T1 + 2*T2*t + 3*T3*t^2
        ! the value of velocity is min/max at t = -T2/(3*T3)
        ! assuming this always lies in the interval [0,T]
        ! TODO: check if this assumption is valid and can be used safely
        do i=1,dof
            print *, bisection_search(displacement(i), v_start(i), v_end(i), v_max(i))
        end do
        print *, ""

    end subroutine primitive

    function bisection_search(disp, td0, td1, vmax)
        real, parameter :: eps = 1e-4
        real, intent(in) :: disp, td0, td1, vmax
        real :: bisection_search
        real :: t, t_lo, t_hi
        integer, parameter :: max_iter = 50 ! limit max number of interations
        integer :: i
        real :: tmp, tmp1, tmp2
        
        ! assuming that the solution always lies in [0.1,10], this 
        ! assumption has been verified by running on different test cases.
        ! TODO: A better method for obtaining the bracket on solution
        t_lo = 0.1
        t_hi = 10.0

        ! bisection method for obtaining a solution to the equation:
        !
        ! |        [3*disp - T(2*td0 + td1)]^2  |
        ! | td0 - ----------------------------- | <= vmax
        ! |       3*(-2T*disp + T*T*(td0 + td1) |
        !
        ! this is the condition which  needs to be satisfied so that
        ! the maximum velocity achieved in the motion primitive does
        ! not exceed the maximum allowed speed of the arm joints
        do i=1,max_iter
            ! TODO: Handle the case when displacement = 0 by considering higher order system
            if (abs(disp) < 1e-2) then
                if (td0 .eq. td1) then
                    print "(a)", 'Identical start and end conditions, ZERO time required'
                    bisection_search = 0.
                    return
                else if (td0 .ne. td1) then
                    print "(a)", 'Motion primitive requires INFINITE acceleration, skipping'
                    bisection_search = -1.
                    return
                end if
            end if

            t = 0.5 * (t_lo + t_hi)
            tmp  = abs(td0 - (1./3)*((3.*disp- t * (2.*td0+td1))**2)/(-2.* t * disp + t * t * (td0 + td1))) - vmax
            tmp1 = abs(td0 - (1./3)*((3.*disp-t_lo*(2.*td0+td1))**2)/(-2.*t_lo*disp + t_lo*t_lo*(td0+td1))) - vmax
            tmp2 = abs(td0 - (1./3)*((3.*disp-t_hi*(2.*td0+td1))**2)/(-2.*t_hi*disp + t_hi*t_hi*(td0+td1))) - vmax
            ! this condition will get triggered only on the very first iteration
            if (tmp1 * tmp2 > 0) then
                print *, 'Solution bracket lost, returning with an error value'
                bisection_search = -1.
                return
            end if
            if (tmp*tmp1 >= 0) t_lo = t
            if (tmp*tmp2 >= 0) t_hi = t
            if (t_hi - t_lo < eps) then
                print *, 'Bisection converged after ',i,' iterations'
                exit
            end if
        end do

        ! inform user if the interation has not yet converged
        if (i .eq. max_iter) then
            print *, 'Iteration limit hit before the solution converged'
        end if

        bisection_search = t_hi
    end function bisection_search

end module planning
