FLAGS=-Wall
arm2d: arm2d.f03 planning.o
	gfortran $(FLAGS) arm2d.f03 planning.o -o arm2d
planning.o: planning.f03
	gfortran -c $(FLAGS) planning.f03
clean:
	rm *.o *.mod arm2d
