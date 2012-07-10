CFLAGS=	-Imongoose -pthread -g

all:
	OS=`uname`; \
	  test "$$OS" = Linux && LIBS="-ldl" ; \
	  $(CC) $(CFLAGS) rt_http.c mongoose/mongoose.c  $$LIBS $(ADD) -o rt_http
