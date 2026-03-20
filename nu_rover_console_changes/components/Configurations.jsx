import React, {useEffect} from 'react'
import { useRecoilState } from 'recoil';
import {
  Card,
  CardBody,
  Typography,
  Switch,
  Input,
  Button
} from "@material-tailwind/react";

import { controlModeAtom } from 'src/recoil/atom/controlModeAtom';
import { receiveFootageAtom } from 'src/recoil/atom/receiveFootageAtom';
import ConfirmationDialog from '../common/ConfirmationDialog';
import { useState } from 'react';


{/** CONTROL */}

const Configurations = () => {
  // Toggle between manual and auto
  const [controlMode, setControlMode] = useRecoilState(controlModeAtom);
  // Toggle camera stream receival
  const [receiveFootage, setReceiveFootage] = useRecoilState(receiveFootageAtom);
  // Toggle Confirmation Dialog
  const [confirmOpen, setConfirmOpen] = React.useState(false);

  const handleControlModeChange = (e) => {
    setConfirmOpen(true);
  }

  const handleReceiveFootageChange = (e) => {
    setReceiveFootage(e.target.checked);
  }

  const [scoopingSpeed, setScoopingSpeed] = useState(5);

  const sendScoopingCommand = (command) => {
    sendScoopingMessage(command);
  };

  const RoboarmOdriveReboot = async () => {
  try {
    const response = await fetch(`http://${import.meta.env.VITE_ORIN_IP}:${import.meta.env.VITE_SCI_PORT}/reboot_odrive`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      }
    });

    const data = await response.json();
    console.log("ODrive reboot response:", data);

  } catch (error) {
    console.error("Failed to reboot ODrive:", error);
  }
};



    return (
        <Card className="!bg-opacity-70 bg-white text-black dark:bg-secondary dark:text-white">
            <CardBody className='flex flex-col gap-4'>
                <Typography variant="h4" className="text-primary font-heading">Configs</Typography>
                <div className='flex gap-2'>
                    <ConfirmationDialog open={confirmOpen} onClose={() => setConfirmOpen(false)}
                                        onConfirm={() => setControlMode(!controlMode)} heading="Switch Control Mode?" />
                    <Typography variant="h5">Drive mode:</Typography>
                    <Typography>Manual</Typography>
                    <Switch
                        id='control-mode-switch'
                        checked={controlMode}
                        onChange={handleControlModeChange}
                        ripple={false}
                        className="h-full w-full checked:bg-primary"
                        containerProps={{
                            className: "w-11 h-6",
                        }}
                        circleProps={{
                            className: "before:hidden left-0.5 border-none",
                        }} />
                    <Typography>Autonomous</Typography>
                </div>
                <div className='flex gap-2'>
                    <Typography variant="h5" >Receive camera stream:</Typography>
                    <Switch
                        id='recieve-footage-switch'
                        checked={receiveFootage}
                        onChange={handleReceiveFootageChange}
                        ripple={false}
                        className="h-full w-full checked:bg-primary"
                        containerProps={{
                            className: "w-11 h-6",
                        }}
                        circleProps={{
                            className: "before:hidden left-0.5 border-none",
                        }} />
                </div>

                <div className="flex flex-col gap-2">
                    <Typography className="text-xl font-bold">ODrive Control</Typography>
                    <Button className="bg-red-600" onClick={RoboarmOdriveReboot}> Reboot ODrive </Button>
                </div>
            </CardBody>
        </Card>
    )
}

export default Configurations