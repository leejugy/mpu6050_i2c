all:$(SRC)
	$(CC) -o $(OBJECT) $^ $(CFLAGS) $(CPPFLAGS) $(LDFLAGS)

clean:
	rm -rf $(OBJECT)