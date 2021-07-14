all: main.exe mainOMP.exe mainOCL.exe

CC=clang++
CFLAGS=-Wall -O2 -g -fopenmp -lOpenCL

valgrind=valgrind

OBJS=stopwatch.o 

%.o: src/%.cpp inc/%.hpp
	$(CC) $(CFLAGS) -c $< -o $@

%OMP.o: src/%OMP.cpp inc/%.hpp
	$(CC) $(CFLAGS) -fopenmp -c $< -o $@

%OCL.o: src/%OCL.cpp inc/%.hpp
	$(CC) $(CFLAGS) -c $< -o $@

main.exe: main.cpp HoughPlane.o $(OBJS)
	$(CC) $(CFLAGS) $< -o $@ HoughPlane.o $(OBJS)
mainOMP.exe: main.cpp HoughPlaneOMP.o $(OBJS)
	$(CC) $(CFLAGS) $< -o $@ HoughPlaneOMP.o $(OBJS)
mainOCL.exe: main.cpp HoughPlaneOCL.o $(OBJS)
	$(CC) $(CFLAGS) $< -o $@ HoughPlaneOCL.o $(OBJS) -lOpenCL

test: main.exe
	$(valgrind) ./$< data/set1.pts 180 90 1000 0.9 set1.ply
	$(valgrind) ./$< data/set2.pts 180 90 1000 0.8 set2.ply
	$(valgrind) ./$< data/set3.pts 120 60 100 0.7 set3.ply
	$(valgrind) ./$< data/set4.pts 60 30 100 0.2 set4.ply
	$(valgrind) ./$< data/set5.pts 90 45 300 0.2 set5.ply
	$(valgrind) ./$< data/set6.pts 150 75 500 0.10 set6.ply
	$(valgrind) ./$< data/Computer.pts 500 250 100 0.15 computer.ply

run: main.exe
	# $(valgrind) ./$< /home/m10805513/FinalExam/data/set1.pts 9 5 10 0.9 set1.ply >log
	# ./$< data/set1.pts 180 90 500 0.9 set1.ply >>log
	# ./$< data/set2.pts 180 90 1000 0.8 set2.ply >>log
	# ./$< data/set3.pts 120 60 100 0.7 set3.ply >>log
	./$< data/set4.pts 120 60 200 0.2 set4.ply >>log
	# ./$< data/set5.pts 90 45 300 0.2 set5.ply >log
	# ./$< data/set6.pts 150 75 500 0.10 set6.ply >>log
	# ./$< data/Computer.pts 500 250 100 0.15 computer.ply

runomp: mainOMP.exe
	# $(valgrind) ./$< /home/m10805513/FinalExam/data/set1.pts 9 5 10 0.9 set1.ply >log
	./$< data/set1.pts 180 90 1000 0.9 set1.ply >log
	# ./$< data/set2.pts 180 90 1000 0.8 set2.ply
	# ./$< data/set3.pts 120 60 100 0.7 set3.ply
	./$< data/set4.pts 60 30 100 0.2 set4.ply >log
	# ./$< data/set5.pts 100 70 120 0.2 set5.ply  >log
	# ./$< data/set6.pts 100 90 200 0.10 set6.ply >log
	# ./$< data/Computer.pts 500 250 100 0.15 computer.ply

clean: 
	rm *.exe *.o *.ply log
	
