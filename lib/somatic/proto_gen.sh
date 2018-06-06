#!/bin/bash
# call this script to update the protoc-generated 
# files after adding messages
# jon scholz 7/8/13

echo "compiling somatic.proto"
protoc-c --c_out=. proto/somatic.proto

# move the files and fix the includes to agree with the move
mv -v proto/somatic.pb-c.c src/somatic.pb-c.c
mv -v proto/somatic.pb-c.h include/somatic.pb-c.h
sed -i -e "s/#include \"proto\\/somatic.pb-c.h\"/#include \"somatic.pb-c.h\"/" src/somatic.pb-c.c

# insert an extern C at the very beginning of the generated file. Usually it'd go inside teh
# include guard, but we do it here to make it easier to close the extern at the end of the file -
# since the opening is outside the guard, we can stick the closing outside the guard as well, in
# particular on the very last line of the file.
sed -i '1i\
#ifdef __cplusplus\
extern "C" {\
#endif\
' include/somatic.pb-c.h

# and close the extern c
echo '#ifdef __cplusplus
}
#endif
' >> include/somatic.pb-c.h

# update the user
echo "done."
