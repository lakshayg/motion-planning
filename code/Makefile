
FLAGS=-Wall
main: main.f03 planning.o
	gfortran $(FLAGS) main.f03 planning.o -o main
planning.o: planning.f03
	gfortran -c $(FLAGS) planning.f03
clean:
	rm *.o *.mod main

