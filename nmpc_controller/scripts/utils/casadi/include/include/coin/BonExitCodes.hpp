#ifndef BonExitCodes_H
#define BonExitCodes_H


namespace Bonmin{
 /** Some error codes for uncatachable errors.*/
enum ErrorCodes{
 ERROR_IN_AMPL_SUFFIXES = 111,
 UNSUPPORTED_CBC_OBJECT/** There is a CbcObject in the model which is not understood by Bonmin.*/
};
}
#endif
