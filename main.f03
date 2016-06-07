program main
    use planning
    implicit none
    real, dimension(2,9) :: vel
    integer :: i, j, x, y

    vel = reshape([ -1., -1.,  &
                    -1.,  0.,  &
                    -1.,  1.,  &
                     0., -1.,  &
                     0.,  0.,  &
                     0.,  1.,  &
                     1., -1.,  &
                     1.,  0.,  &
                     1.,  1.], [2,9])

    !call primitive([1.,1.], [1.,1.], [-1.,-1.], [2.,2.])
    do i=1,9
        do j=1,9
            do x=-1,1
                do y=-1,1
                    print "(a)", 'Problem description:'
                    print "(a,2f8.3)", 'Displacement: ', [1.*x, 1.*y]
                    print "(a,2f8.3,4x,2f8.3)", 'Velocities at boundaries: ', vel(:,i), vel(:,j)
                    call primitive([1.*x,1.*y], vel(:,i), vel(:,j), [2.5,2.5])
                end do
            end do
        end do
    end do
    !call primitive([50.,1.], [1.,0.], [0.,1.], [10.,1.1])
    !call primitive([1.,1.], [-1.,0.], [0.,1.], [1.5,1.5])
    !call primitive([1.,0.], [1.,0.1], [0.1,1.], [1.5,1.5])
    !call primitive([1.,0.], [-1.,0.], [0.,1.], [1.5,1.5])
    !call primitive([-1.,1.], [1.,0.], [0.,-1.], [1.5,1.5])
    !call primitive([1.,-1.], [-1.,0.], [0.,1.], [1.5,1.5])
end program main
