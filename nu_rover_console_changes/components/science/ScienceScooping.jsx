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
                <div className="grid gap-4">
                    <Typography className="text-xl" style={{ fontWeight: 'bold' }}><strong>Scooping Speed</strong></Typography>
                    <div className="flex gap-0.5">
                        <Input className="text-black dark:text-white" min='0' max='50' type="number" value={scoopingSpeed} onChange={(e) => setScoopingSpeed(e.target.value)} />
                        <Button className="border bg-green-700" onClick={() => sendScoopingCommand(`SET_SPEED_${scoopingSpeed}`)}>Apply</Button>
                    </div>
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