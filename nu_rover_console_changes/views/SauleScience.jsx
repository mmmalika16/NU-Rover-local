import React, {useEffect} from 'react'
import { Card, CardBody } from "@material-tailwind/react";
import { RTCPeerCtx } from 'src/contexts/RTCPeerContext';
import Configurations from 'src/components/science/ScienceScooping';
import Ilmenite from 'src/components/science/Ilmenite';


import { useRecoilValue, useRecoilState } from 'recoil';
import { controlModeAtom } from 'src/recoil/atom/controlModeAtom';
import CameraView from './CameraView';

import { LoraCtx } from 'src/contexts/LoraContext';

const CommandControl = () => {
  const { remoteStream } = RTCPeerCtx();
  const { serialState, sendTextMessage } = LoraCtx();

  // Toggle between manual and auto
  const controlMode = useRecoilValue(controlModeAtom);

  // useEffect(() => {
  //   console.log("Control mode changed:", controlMode);
  //   if (serialState && !controlMode) {
  //     sendTextMessage("AUTONOMOUS_OFF\n");
  //     console.log("AUTONOMOUS_OFF");
  //   } else if (serialState && controlMode) {
  //     sendTextMessage("AUTONOMOUS_ON\n");
  //     console.log("AUTONOMOUS_ON");
  //   }
  // }, [controlMode]);

  return (
    <div className='mt-4 grid grid-cols-12 gap-4'>
      <div className='col-span-3'>
        <div className='flex flex-col gap-4 mb-4'>
          <Configurations />
        </div>
        <div className='flex flex-col gap-4'>
          <Ilmenite />
        </div>
      </div>

      <div className='col-span-8'>
        <Card className='bg-secondary'>
          <CardBody>
            <CameraView stream={remoteStream}/>
          </CardBody>
        </Card>
      </div>
    </div>
  )
}

export default CommandControl