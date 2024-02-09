/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * $Id: TableDataRow.java 7377 2020-08-07 14:40:53Z Fabrice $
 * @LastChange $Date: 2020-08-07 16:40:53 +0200 (ven., 07 août 2020) $
 * 
 * Demo Application for PCANBasic JAVA JNI Interface.
 *
 * Copyright (C) 2001-2020  PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * PCAN is a registered Trademark of PEAK-System Germany GmbH
 *
 * Author: 		 Jonathan Urban/Uwe Wilhelm/Fabrice Vergnaud
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Fabrice Vergnaud <f.vergnaud@peak-system.com>
 */
package peak.can;

import peak.can.basic.TPCANMessageType;
import peak.can.basic.TPCANMsg;
import peak.can.basic.TPCANMsgFD;
import peak.can.basic.TPCANTimestamp;
import peak.can.basic.TPCANTimestampFD;

/**
 * The TableDataRow class is a structure to store all provided info by a CAN Message
 *
 * @version 1.10
 * @LastChange $Date: 2020-08-07 16:40:53 +0200 (ven., 07 août 2020) $
 * @author Jonathan Urban/Uwe Wilhelm/Fabrice Vergnaud
 *
 * @Copyright (C) 1999-2014  PEAK-System Technik GmbH, Darmstadt
 * more Info at http://www.peak-system.com
 */
public class TableDataRow
{

    //Private fields
    private TPCANTimestamp rcvTime;
    private TPCANTimestampFD rcvTimeFd;
    private int counter;
    private TPCANMsg message;
    private TPCANMsgFD messageFd;

    /**
     * Gets number of times the CAN Message was readed
     * @return number of times
     */
    public int getCounter()
    {
        return counter;
    }

    /**
     * Sets number of times the CAN Message was readed
     * @param counter number of times the CAN Message was readed
     */
    public void setCounter(int counter)
    {
        this.counter = counter;
    }

    /**
     * Sets wrapped TPCANMsg
     * @param message wrapped TPCANMsg
     */
    public void setMessage(TPCANMsg message)
    {
        this.message = message;
    }
    
    /**
     * Sets wrapped TPCANMsgFD
     * @param messageFd wrapped TPCANMsgFD
     */
    public void setMessage(TPCANMsgFD messageFd)
    {
        this.messageFd = messageFd;
    }
    
    /**
     * Sets wrapped TPCANTimestamp
     * @param rcvTime wrapped TPCANTimestamp
     */
    public void setRcvTime(TPCANTimestamp rcvTime)
    {
        this.rcvTime = rcvTime;
    }
    /**
     * Sets wrapped TPCANTimestamp
     * @param rcvTimeFd wrapped TPCANTimestamp
     */
    public void setRcvTime(TPCANTimestampFD rcvTimeFd)
    {
        this.rcvTimeFd = rcvTimeFd;
    }
    
    
    
    public String getMsgType() {
        String result;
        
        result = "";
        if (message != null) {
            if (message.getType() == TPCANMessageType.PCAN_MESSAGE_STANDARD.getValue())
                result = "Standard";
            else if (message.getType() == TPCANMessageType.PCAN_MESSAGE_EXTENDED.getValue())
                result = "Extended";
            else
                result = Byte.toString(message.getType());
        }
        else if (messageFd != null) {
            
            if (messageFd.getTypeEnum().contains(TPCANMessageType.PCAN_MESSAGE_STATUS))
                result = "Status";
            else if (messageFd.getTypeEnum().contains(TPCANMessageType.PCAN_MESSAGE_STANDARD))
                result = "Standard";
            else if (messageFd.getTypeEnum().contains(TPCANMessageType.PCAN_MESSAGE_EXTENDED))
                result = "Extended";
            else 
                result = Byte.toString(messageFd.getType());
            if (messageFd.getTypeEnum().contains(TPCANMessageType.PCAN_MESSAGE_FD))
                result += " (FD)";
            
        }
        return result;
    }
    
    public int getMsgLength() {
        if (message != null) {
            return message.getLength();
        }
        else if (messageFd != null) {
            return messageFd.getLengthFromDLC();
        }
        return 0;
    }
    
    public int getMsgId() {        
        if (message != null) {
            return message.getID();
        }
        else if (messageFd != null) {
            return messageFd.getID();
        }
        return 0;
    }
    
    public byte[] getMsgData() {        
        if (message != null) {
            return message.getData();
        }
        else if (messageFd != null) {
            return messageFd.getData();
        }
        return null;
    }
    
    public String getRcvTimeAsString() {        
        if (rcvTime != null) {
            return String.valueOf(rcvTime.getMillis()) + "." + String.valueOf(rcvTime.getMicros());
        }
        else if (rcvTimeFd != null) {
            return String.valueOf(rcvTimeFd.getValue());
        }
        return null;
    }
}
