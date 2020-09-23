#ifndef READ_FUNCTION_H
#define READ_FUNCTION_H

#include <iostream>
#include <fstream>
#include <string>

#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>


namespace Utils
{
  /** Reads a function from an input stream.
   * Memory for _x and _y is allocated, and _n is set, based
   * on the number of data points found in the stream _in. _x and _y should NOT point
   * to allocated memory before being passed to ReadFunction() because the memory will
   * be lost (a memory leak). the multiplicity and dimensions can also be given, which specify
   * how many columns are expected to be coordinates (dimension) and how many are values (multiplicity). so, for example, a
   * complex function f(x), has _multiplicity = 2 and _dimension = 1.
   * \param _in           input stream to read data from. data is read while state of _in is good
   * \param _x            pointer through which coordinates are returned
   * \param _y            pointer through which values are returned
   * \param _n            pointer through which number of points in each dimension is returned
   * \param _dimensions   specifies the number of columns that are coordinates
   * \param _multiplicity specifies the number of columns that make up a single value of the function
   * */
  template < typename ArgType, typename ValType >
  void ReadFunction(std::istream &_in, ArgType  *&_x, ValType *&_y, int *&_n, int _dimensions = 1, int _multiplicity = 1)
  {

    ArgType *xbuffer1, *xbuffer2;
    ValType *ybuffer1, *ybuffer2;

    //////////////////////////////
    //  READ IN DATA FROM FILE  //
    //////////////////////////////

    const int chunck = 1000;
    int buffsize,buffsize_old;
    int i,j;
    int n;

    std::string line;
    

    buffsize = chunck;

    xbuffer1 = new ArgType[buffsize*_dimensions  ];
    ybuffer1 = new ValType[buffsize*_multiplicity];

    i = 0;
    while( getline( _in, line ) )
    {
      // if line is blank, skip it
      if( line == "" )
        continue;
      std::vector<std::string> columns;
      boost::char_separator<char> sep(" \t");
      boost::tokenizer<boost::char_separator<char>> toks(line,sep);
      for( auto it = toks.begin(); it != toks.end(); ++it )
        columns.push_back( *it );

      // if first column starts with a #, this line is a comment
      // and we should skip it.
      if( columns.size() > 0 && columns[0].find('#') == 0 )
        continue;

      if(i == buffsize - 1)  // this "grows" the buffer
      {
        buffsize_old = buffsize;
        buffsize = buffsize + chunck;

        xbuffer2 = new ArgType[buffsize*_dimensions  ];
        std::copy( xbuffer1, xbuffer1 + buffsize_old*_dimensions, xbuffer2);
        delete[] xbuffer1;

        ybuffer2 = new ValType[buffsize*_multiplicity];
        std::copy( ybuffer1, ybuffer1 + buffsize_old*_multiplicity, ybuffer2);
        delete[] ybuffer1;

        xbuffer1 = xbuffer2;
        ybuffer1 = ybuffer2;
      }

      for(j = 0; j < _dimensions; j++)
        xbuffer1[i*_dimensions + j]   = boost::lexical_cast<ArgType>(columns[j]);
      for(j = 0; j < _multiplicity; j++)
        ybuffer1[i*_multiplicity + j] = boost::lexical_cast<ValType>(columns[j+_dimensions]);

      i++;
    }

    n = i;


    /** \todo add actual multi-coordinate support */

    _n = new int[_dimensions];
    _x = new ArgType[n * _dimensions  ];
    _y = new ValType[n * _multiplicity];

    for(j = 0; j < n * _dimensions  ; j++)
      _x[j] = xbuffer1[j];
    for(j = 0; j < n * _multiplicity; j++)
      _y[j] = ybuffer1[j];
    for(j = 0; j < _dimensions; j++)
      _n[j] = n;

    delete[] xbuffer1;
    delete[] ybuffer1;

    }

  /** specialized version of ReadFunction that takes an integer for _n (rather than an integer array)
   * and reads in a 1D function
   */
  template < typename ArgType, typename ValType >
  void ReadFunction(std::istream &_in, ArgType  *&_x, ValType *&_y, int &_n, int _dimensions = 1, int _multiplicity = 1)
  {
    int *n;
    ReadFunction(_in, _x, _y, n, _dimensions, _multiplicity);
    _n = n[0];
    delete[] n;
  }
}


#endif
