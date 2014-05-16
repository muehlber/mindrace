
all: test

mindrace: mindrace.c ThinkGearStreamParser.c
	gcc -c ThinkGearStreamParser.c -o ThinkGearStreamParser.o
	gcc -c mindrace.c -o mindrace.o
	gcc ThinkGearStreamParser.o mindrace.o -o mindrace

test: MindWave
	./mindrace

spelling: README
	ispell -b -d british -p ./mindrace.dict README;

clean:
	rm -f ThinkGearStreamParser.o
	rm -f mindrace.o
	rm -f README.bak

distclean: clean
	rm -f mindrace


