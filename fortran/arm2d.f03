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

    do i=1,9
        do j=1,9
            do x=-1,1
                do y=-1,1
                    call primitive([1.*x,1.*y], vel(:,i), vel(:,j), [2.5,2.5])
                end do
            end do
        end do
    end do
end program main
