import React from 'react'
import {
    Navbar
} from "@material-tailwind/react";
import {
    Cog6ToothIcon,
    CommandLineIcon,
    CameraIcon,
    RocketLaunchIcon,
    BeakerIcon,
    Battery100Icon,
    WifiIcon,
    ServerStackIcon,
    SunIcon,
    MoonIcon,
    BoltIcon,
} from "@heroicons/react/24/solid";
import { AiOutlineRobot } from 'react-icons/ai';
import { BsController } from 'react-icons/bs';
import { useRecoilValue, useRecoilState } from 'recoil';
import { NavLink } from 'react-router-dom';

import logo from '/logo.png'
import { controlModeAtom } from 'src/recoil/atom/controlModeAtom';
import { connectionTypeAtom } from 'src/recoil/atom/connectionTypeAtom';
import { themeAtom } from 'src/recoil/atom/themeAtom';

const NavControlBar = () => {

    const controlMode = useRecoilValue(controlModeAtom);
    const connectionType = useRecoilValue(connectionTypeAtom);
    const [theme, setTheme] = useRecoilState(themeAtom);

    const toggleTheme = () => {
        const newTheme = theme === 'dark' ? 'light' : 'dark';
        setTheme(newTheme);
    };

    return (
        <Navbar className="!bg-opacity-70 !backdrop-blur-0 !backdrop-saturate-100 border-none bg-white text-black dark:bg-secondary dark:text-white h-max max-w-full py-2 px-4 lg:px-8 lg:py-4 grid grid-cols-3">
            <div className='flex items-center'>
                <img className='h-16' src={logo} alt='Logo' />
                <div className='ml-4'>
                    <h4>Nazarbayev <span className='text-primary'>University<br />Rover</span> Team</h4>
                </div>
            </div>
            <div className="h-16 flex items-center justify-center">
                <div className="flex items-center gap-4">
                    <div className="mr-4">
                        <ul className="mb-4 mt-2 flex gap-2 lg:mb-0 lg:mt-0 lg:items-center lg:gap-6">
                            <li >
                                <NavLink to="/" className={({ isActive }) => `transition-colors duration-200 hover:text-primary ${isActive ? "text-primary" : ""}` }>
                                    <RocketLaunchIcon className="h-8 w-8" />
                                </NavLink>
                            </li>
                            <li>
                                <NavLink to="/command" className={({ isActive }) => `transition-colors duration-200 hover:text-primary ${isActive ? "text-primary" : ""}` }>
                                    <CommandLineIcon className="h-8 w-8" />
                                </NavLink>
                            </li>
                            {/* <li>
                                <NavLink to="/camera" className={({ isActive }) => `transition-colors duration-200 hover:text-primary ${isActive ? "text-primary" : ""}` }>
                                    <CameraIcon className="h-8 w-8" />
                                </NavLink>
                            </li> */}
                            {/*<li>
                                <NavLink to="/lab" className={({ isActive }) => `transition-colors duration-200 hover:text-primary ${isActive ? "text-primary" : ""}` }>
                                    <BeakerIcon className="h-8 w-8" />
                                </NavLink>
                            </li>
                            */}
                            <li>
                                <NavLink to="/science" className={({ isActive }) => `transition-colors duration-200 hover:text-primary ${isActive ? "text-primary" : ""}` }>
                                    <BoltIcon className="h-8 w-8" />
                                </NavLink>
                            </li>
                            <li>
                                <NavLink to="/settings" className={({ isActive }) => `transition-colors duration-200 hover:text-primary ${isActive ? "text-primary" : ""}` }>
                                    <Cog6ToothIcon className="h-8 w-8" />
                                </NavLink>
                            </li>
                        </ul>
                    </div>
                </div>
            </div>
            <div className='flex items-center justify-end'>
                <button
                    onClick={toggleTheme}
                    aria-label="Toggle theme"
                    className="mr-4 rounded-md border border-transparent hover:border-primary p-2 transition-colors"
                    title={theme === 'dark' ? 'Switch to light theme' : 'Switch to dark theme'}
                >
                    {theme === 'dark' ? (
                        <SunIcon className="h-6 w-6" />
                    ) : (
                        <MoonIcon className="h-6 w-6" />
                    )}
                </button>
                <div className='mr-4 flex items-center'>
                    <p className='mr-2'>100 %</p>
                    <Battery100Icon className="h-8 w-8" />
                </div>
                <div className='mr-4 flex items-center'>
                    {
                        !controlMode ? (
                            <div className=''>
                                <BsController className="h-8 w-8" />
                            </div>
                        ) : (
                            <div className=''>
                                <AiOutlineRobot className="h-8 w-8" />
                            </div>
                        )
                    }
                </div>
                {
                    connectionType === 'proxy' ? (
                        <div className=''>
                            <ServerStackIcon className="h-8 w-8" />
                        </div>
                    ) : (
                        <div className=''>
                            <WifiIcon className="h-8 w-8" />
                        </div>
                    )
                }
            </div>
        </Navbar>
    );
}

export default NavControlBar