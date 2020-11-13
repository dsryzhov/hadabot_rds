#ifndef __POS_CONTROLLER_RM_
#define __POS_CONTROLLER_RM_

#include "PosController.h"

class PosControllerRM : public PosController {
public:
    PosControllerRM();
    ~PosControllerRM();

    virtual MotionType selectNeededMotionType() override;

    virtual void MoveStep(int dir ) override;
      

protected:

};

#endif
