  $ $TESTDIR/interp-cli
  interp-cli - linked against libInterp version 
  usage: /home/cclark/Code/sync/projects/libInterp/build/interp-cli [OPTIONS]
  
  Allowed options:
    -h [ --help ]                 print help message
    -b [ --batch ]                output in 'batch' mode
    -m [ --method ] arg (=spline) interpolation method.
    -l [ --list ]                 list available interpolation methods.
    --interp-data                 file containing data to be interpolated from.
    --x-values                    file containing x values to interpolate to.
    --output-file                 file to write inteprolated data to.
  
  
  Reads x-y pairs from a file and interpolates to x values listed in another file.
  The interpoalted data (data that is interpolated from) is contained in a gnuplot-style text file, with each x-y pair on a new line, separated by white space.The x points to interpoalte to are contained in plain text file, with each value on a new line.
  Notes:
  \tthe x values to be interpolated to can also be stored in a gnuplot-style text file. If the file containes more than one column, only the first will be used. (esc)
  
  
  [1]
  $ $TESTDIR/interp-cli -l
  interp-cli - linked against libInterp version 
  \tlinear (esc)
  \tspline (esc)
  \tmonotonic (esc)
  [1]
  $ echo '1 2\n2 3\n3 4' >> input.txt
  $ echo '0.5\n1.5\n2.5\n3.5' >> x.txt
  $ cat input.txt
  1 2
  2 3
  3 4
  $ cat x.txt
  0.5
  1.5
  2.5
  3.5
  $ $TESTDIR/interp-cli -m linear input.txt x.txt output.txt
  $ cat output.txt
  0.5 0
  1.5 2.5
  2.5 3.5
  3.5 0
  $ $TESTDIR/interp-cli -m spline input.txt x.txt output.txt
  $ cat output.txt
  0.5 0
  1.5 2.5
  2.5 3.5
  3.5 0
  $ $TESTDIR/interp-cli -m monotonic input.txt x.txt output.txt
  $ cat output.txt
  0.5 0
  1.5 2.52083
  2.5 3.5
  3.5 0
