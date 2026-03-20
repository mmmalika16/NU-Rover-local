import {
  Avatar,
  Button,
  ButtonGroup,
  Card,
  CardBody,
  Input,
  List,
  ListItem,
  ListItemPrefix,
  Typography
} from "@material-tailwind/react";

import Papa from "papaparse";
import { useState, useEffect } from "react";

import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis
} from "recharts";

import { useRecoilState, useRecoilValue } from "recoil";

import elementsList from "src/db/elements.json";
import { graphDatapointAtom } from "src/recoil/atom/graphDatapointAtom";
import { phDataAtom } from "src/recoil/atom/phDataAtom";

import rockGreen from "/assets/rockGreen.png";

import { WebsocketCtx } from "../contexts/WebsocketContext";

import LiveCameraPlayer from "src/components/common/LiveCameraPlayer"; // ✅ missing import

const Science = ({ stream }) => {

  const [mainIndex, setMainIndex] = useState(0);
  const [humidity, setHumidity] = useState(null);

  useEffect(() => {
    if (!stream) return;

    if (mainIndex >= stream.length) {
      setMainIndex(0);
    }
  }, [stream, mainIndex]);

  const handleCameraChange = (index) => {
    setMainIndex(index);
  };

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

    <div className="mt-4 flex flex-col gap-4">

      {/* Science Title Card */}
      <div>
        <Card className="!bg-opacity-70 bg-white text-black dark:bg-secondary dark:text-white">
          <CardBody className="grid gap-4">
            <Typography variant="h4" className="mb-2 text-primary font-heading">
              Science
            </Typography>
          </CardBody>
        </Card>
      </div>

      <Typography variant="h4" className="mb-3 text-primary font-heading">
        Camera
      </Typography>

      {/* Camera Grid */}
      <div className="grid grid-cols-2 gap-3">

        {stream?.map((s, index) => {

          if (!s) return null;

          return (
            <div
              key={index}
              onClick={() => handleCameraChange(index)}
              className={`cursor-pointer rounded-lg overflow-hidden transition-all duration-200 ${
                index === mainIndex
                  ? "ring-4 ring-blue-500 ring-offset-2"
                  : "opacity-80 hover:opacity-100"
              }`}
            >
              <LiveCameraPlayer mediaStream={s} />
            </div>
          );

        })}

      </div>

      {/* Ilmenite Card */}
      <Card className="!bg-opacity-70 bg-white text-black dark:bg-secondary dark:text-white gap-2">
        <CardBody>

          <Typography variant="h4" className="mb-4 text-primary font-heading"> Ilmenite</Typography>

          <div className="flex items-center mb-2 gap-2">
            <Typography variant="h5"> Ice Concentration: {humidity}</Typography>
          </div>

        </CardBody>
      </Card>

    </div>

  );
};

export default Science;