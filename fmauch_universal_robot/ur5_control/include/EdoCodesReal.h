#include <cstdint>
#include <string>

#ifndef PROJECT_EDOCODESREAL_H
#define PROJECT_EDOCODESREAL_H

class EdoCodesReal
{
public:
    EdoCodesReal();
    ~EdoCodesReal();
    static std::string getErrorMessages(int8_t error_num);
};
#endif //PROJECT_EDOCODESREAL_H
