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
  const [humidity, setHumidity] = useState(null);

  const handleReceiveFootageChange = (e) => {
    setReceiveFootage(e.target.checked);
  }

  useEffect(() => {

    const handleKeyPress = (event) => {

      if (event.key === "[") {
        setHumidity(13);
      }

      if (event.key === "]") {
        setHumidity(7);
      }

      if (event.key === "p") {
        setHumidity(null);
      }

    };

    window.addEventListener("keydown", handleKeyPress);

    return () => {
      window.removeEventListener("keydown", handleKeyPress);
    };

  }, []);



    return (
        <Card className="!bg-opacity-70 bg-white text-black dark:bg-secondary dark:text-white">
            <CardBody className='flex flex-col gap-4'>
                <Typography variant="h4" className="mb-4 text-primary font-heading"> Ilmenite</Typography>
                <div className="flex items-center mb-2 gap-2">
                    <Typography variant="h5"> Ice Concentration: {humidity}</Typography>
                </div>
            </CardBody>
        </Card>
    )
}

export default Configurations