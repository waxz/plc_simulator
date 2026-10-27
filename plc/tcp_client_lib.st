USING Siemens.Simatic.Communication.OUC;
USING System;
USING System.Serialization; // For handling byte-to-integer conversions


FUNCTION TcpBlockMove
    VAR_IN_OUT
        source : ARRAY[*] OF BYTE;
        dest : ARRAY[*] OF BYTE;
    END_VAR
    VAR_INPUT
        srcOffset : UDINT;
        dstOffset : UDINT;
        count : UDINT;
    END_VAR
    VAR_TEMP
        I : UDINT;
        srcIdx : DINT;
        dstIdx : DINT;
    END_VAR

    IF count = UDINT#0 THEN
        RETURN;
    END_IF;

    srcIdx := LOWER_BOUND(source, 1) + TO_DINT(srcOffset);
    dstIdx := LOWER_BOUND(dest, 1) + TO_DINT(dstOffset);

    FOR I := UDINT#0 TO count - UDINT#1 DO
        dest[dstIdx] := source[srcIdx];
        srcIdx := srcIdx + DINT#1;
        dstIdx := dstIdx + DINT#1;
    END_FOR;
END_FUNCTION

CLASS TcpClient
    VAR CONSTANT
        MAX_SEND_LENGTH : UINT := UINT#256;
        MAX_RECEIVE_LENGTH : UINT := UINT#256;
    END_VAR

    VAR PUBLIC
        IsConfigured : BOOL := FALSE;
        IsConnected : BOOL := FALSE;

        ConnectDone, ConnectError, ConnectBusy : BOOL;
        ConnectExecute : BOOL;
        ConnectStatus, LastConnectStatus : WORD;
        ConnectAttemptCount, ConnectDoneCount, ConnectErrorCount : UDINT;

        SendDone, SendError, SendBusy : BOOL;
        SendExecute : BOOL;
        SendStatus, LastSendStatus : WORD;
        SendDoneCount, SendErrorCount, DroppedSendCount : UDINT;

        ReceiveNewData, ReceiveError, ReceiveBusy : BOOL;
        ReceiveStatus, LastReceiveStatus : WORD;
        ReceiveLength :INT;
        ReceiveCount, ReceiveErrorCount : UDINT;
        ReceiveBuffer : ARRAY [0..MAX_RECEIVE_LENGTH - UINT#1] OF BYTE;
    END_VAR

    VAR PRIVATE
        config : ConnectionConfiguration;
        connect : Connect;
        send : TcpSend;
        receive : TcpReceive;
        sendTimer : IEC.TON;
        sendPeriod : TIME := T#1S;
        sendBuffer : ARRAY [0..MAX_SEND_LENGTH - UINT#1] OF BYTE;
        sendLength : UDINT := UDINT#0;
        sendQueued : BOOL := FALSE;
        prevConnectExecute : BOOL := FALSE;
           retryTimer : IEC.TON;
    retryDelay : TIME := T#2S;
    forceExecuteLow : BOOL := FALSE;

    END_VAR

    METHOD PUBLIC Configure
        VAR_INPUT
            remoteAddress : Siemens.Simatic.Communication.IPConfiguration.IPv4;
            remotePort : UINT;
            connectionID : WORD := WORD#1;
            hardwareID : UINT := UINT#64;
            period : TIME := T#1S;
        END_VAR

        IF NOT IsConfigured THEN
            config.HardwareID := hardwareID;
            config.ConnectionID := connectionID;
            config.RemoteAddress := remoteAddress;
            config.RemotePort := remotePort;
            sendPeriod := period;
            IsConfigured := TRUE;
        END_IF;
    END_METHOD

    METHOD PUBLIC QueueSend
        VAR_INPUT
            dataLength : UDINT;
        END_VAR
        VAR_IN_OUT
            data : ARRAY[*] OF BYTE;
        END_VAR

        IF (dataLength + UDINT#8) > TO_UDINT(MAX_SEND_LENGTH) THEN
            DroppedSendCount := DroppedSendCount + UDINT#1;
            RETURN;
        END_IF;

        IF SendBusy THEN
            DroppedSendCount := DroppedSendCount + UDINT#1;
            RETURN;
        END_IF;

        sendLength := dataLength + UDINT#8;

        // Fixed magic marker 16#12345678, followed by little-endian UDINT payload length.
        sendBuffer[0] := BYTE#16#12;
        sendBuffer[1] := BYTE#16#34;
        sendBuffer[2] := BYTE#16#56;
        sendBuffer[3] := BYTE#16#78;
        sendBuffer[4] := TO_BYTE(TO_DWORD(dataLength) AND DWORD#16#FF);
        sendBuffer[5] := TO_BYTE(SHR(TO_DWORD(dataLength), USINT#8) AND DWORD#16#FF);
        sendBuffer[6] := TO_BYTE(SHR(TO_DWORD(dataLength), USINT#16) AND DWORD#16#FF);
        sendBuffer[7] := TO_BYTE(SHR(TO_DWORD(dataLength), USINT#24) AND DWORD#16#FF);

        TcpBlockMove(
            source := data,
            dest := sendBuffer,
            srcOffset := UDINT#0,
            dstOffset := UDINT#8,
            count := dataLength
        );

        sendQueued := TRUE;
    END_METHOD

    METHOD PUBLIC Run 

        VAR_TEMP
            recv_len : udint;
            END_VAR

// in Run():
IF ConnectError THEN
    forceExecuteLow := TRUE;
    retryTimer(IN := TRUE, PT := retryDelay);
ELSIF retryTimer.Q THEN
    forceExecuteLow := FALSE;
    retryTimer(IN := FALSE);
END_IF;

ConnectExecute := NOT IsConnected AND NOT connect.busy AND NOT forceExecuteLow;

        
        IF NOT IsConfigured THEN
            RETURN;
        END_IF;

        // ConnectExecute := NOT IsConnected AND NOT connect.busy;
        IF ConnectExecute AND NOT prevConnectExecute THEN
            ConnectAttemptCount := ConnectAttemptCount + UDINT#1;
        END_IF;
        prevConnectExecute := ConnectExecute;

        // connect(
        // execute := NOT IsConnected AND NOT connect.busy,
        // connectionID := config.ConnectionID,
        // connectionConfiguration := config,
        // done => ConnectDone,
        //     busy => ConnectBusy,
        //     error => ConnectError,
        //     status => ConnectStatus
        // );


        connect(
            execute := ConnectExecute,
            connectionID := config.ConnectionID,
            connectionConfiguration := config,
            done => ConnectDone,
            busy => ConnectBusy,
            error => ConnectError,
            status => ConnectStatus
        );

        IF ConnectStatus <> WORD#16#7000 THEN
            LastConnectStatus := ConnectStatus;
        END_IF;

        IF ConnectDone THEN
            ConnectDoneCount := ConnectDoneCount + UDINT#1;
            IsConnected := TRUE;
        END_IF;

        IF ConnectError THEN
            ConnectErrorCount := ConnectErrorCount + UDINT#1;
        END_IF;

        sendTimer(IN := IsConnected AND sendQueued AND NOT sendTimer.Q, PT := sendPeriod);
        SendExecute := IsConnected AND sendQueued AND sendTimer.Q AND NOT send.busy;

        send(
            execute := SendExecute,
            connectionID := config.ConnectionID,
            length := sendLength,
            data := sendBuffer,
            done => SendDone,
            busy => SendBusy,
            error => SendError,
            status => SendStatus
        );

        IF SendStatus <> WORD#16#7000 THEN
            LastSendStatus := SendStatus;
        END_IF;

        IF SendDone THEN
            SendDoneCount := SendDoneCount + UDINT#1;
        END_IF;

        IF SendError THEN
            SendErrorCount := SendErrorCount + UDINT#1;
        END_IF;
        receive(
            enable := IsConnected,
            connectionID := config.ConnectionID,
            adhocMode := TRUE,
            data := ReceiveBuffer,
            length := MAX_RECEIVE_LENGTH,

            newDataReceived => ReceiveNewData,
            receivedLength =>  recv_len,
            busy => ReceiveBusy,
            error => ReceiveError,
            status => ReceiveStatus
        );

        IF ReceiveNewData THEN
            ReceiveLength := TO_INT(recv_len);
        ELSE
            ReceiveLength := INT#0;
        END_IF;

        IF ReceiveStatus <> WORD#16#7000 THEN
            LastReceiveStatus := ReceiveStatus;
        END_IF;

        IF ReceiveNewData THEN
            ReceiveCount := ReceiveCount + UDINT#1;
        END_IF;

        IF ReceiveError THEN
            ReceiveErrorCount := ReceiveErrorCount + UDINT#1;
        END_IF;
    END_METHOD
END_CLASS


CLASS TCP_StreamParser1 

    VAR CONSTANT
        BUFFER_LENGTH : INT := 4095;
        PAYLOAD_LENGTH : INT := 2048;

        // Instance of your TRCV or communication interface
        // e.g., MyTrcvInstance : TRCV;
    END_VAR

    VAR
        // Adjust buffer size depending on your largest expected payload + 8 byte header
        Buffer : Array[0..BUFFER_LENGTH] Of Byte;
        BufferLength : Int := 0; // Tracks current number of valid bytes in Buffer
        
    END_VAR

    VAR PUBLIC
        // Parsing variables
        // i : Int := 0;
        MagicFound : Bool := False;
        MarkerIndex : Int := 0;
        PayloadLength : DInt := 0;
        TotalPacketSize : Int := 0;
        
        // Output/Export extracted data
        PayloadReady : Bool := False;
        ExtractedPayload : Array[0..PAYLOAD_LENGTH] Of Byte;
        ExtractedLength : Int := 0;
    END_VAR

    METHOD PUBLIC Execute
        VAR
            MoveIndex : Int;
              
        END_VAR
        VAR_IN_OUT
           // Temp array to capture immediate cycle data from network
            RxTemp : Array[*] Of Byte;
            RxCount : Int;
        END_VAR
        VAR_TEMP
            i : int;

        END_VAR
        
        PayloadReady := False; // Reset flag every cycle

        // 1. SIMULATED/ACTUAL RECV CALL: Fetch raw data from network interface
        // Replace this block with your actual TRCV logic
        // TRCV_Instance(RCV:=RxTemp, ...NDR=>DataReady, LEN=>RxCount);
        
        // If data arrived this cycle, append it to our main stream buffer
        IF RxCount > 0 THEN
            // Ensure we don't overflow our main stream buffer
            IF (BufferLength + RxCount) <= 4096 THEN
                // Copy temporary network chunk to the end of our stream buffer
                FOR i := 0 TO RxCount - 1 DO
                    Buffer[BufferLength + i] := RxTemp[i];
                END_FOR;
                BufferLength := BufferLength + RxCount;
            ELSE
                // Handle Buffer Overflow Error here (e.g., reset buffer)
                BufferLength := 0;
            END_IF;
        END_IF;

        // 2. PARSING LOOP: Process buffer continuously until no complete packets remain
        WHILE BufferLength >= 8 DO // Must have at least 8 bytes (4B Magic + 4B Length)
            
            MagicFound := False;
            
            // Look for the 4-byte Magic Marker: 0x12345678
            FOR i := 0 TO (BufferLength - 4) DO
                IF Buffer[i] = TO_BYTE(16#12) AND Buffer[i+1] = TO_BYTE(16#34) AND Buffer[i+2] = TO_BYTE(16#56) AND Buffer[i+3] = TO_BYTE(16#78) THEN
                    MagicFound := True;
                    MarkerIndex := i;
                    EXIT; // Found the earliest marker, break FOR loop
                END_IF;
            END_FOR;

            // If magic marker is not anywhere in the buffer, discard junk data safely
            IF NOT MagicFound THEN
                // Keep only the last 3 bytes in case the magic word was split between frames
                IF BufferLength > 3 THEN
                    FOR i := 0 TO 2 DO
                        Buffer[i] := Buffer[BufferLength - 3 + i];
                    END_FOR;
                    BufferLength := 3;
                END_IF;
                EXIT; // Break the WHILE loop, wait for more data
            END_IF;

            // If the magic marker was found deeper in the stream, slide buffer to align to it
            IF MarkerIndex > 0 THEN
                FOR i := 0 TO (BufferLength - 1 - MarkerIndex) DO
                    Buffer[i] := Buffer[MarkerIndex + i];
                END_FOR;
                BufferLength := BufferLength - MarkerIndex;
            END_IF;

            // Double check if we still have the full 8 bytes header after realignment
            IF BufferLength < 8 THEN
                EXIT; // Wait for next cycle data
            END_IF;

            // 3. EXTRACT LENGTH: Deserialize 4 bytes following the magic marker (little-endian)
            // Reconstructs DInt from Buffer[4..7]
            // PayloadLength := TO_DINT(SHL(TO_DWORD(Buffer[4]), UINT#24) OR 
            //                  SHL(TO_DWORD(Buffer[5]), UINT#16) OR 
            //                  SHL(TO_DWORD(Buffer[6]), UINT#8)  OR 
            //                  TO_DWORD(Buffer[7]));

            // Combined safely into a DINT
PayloadLength := TO_DINT(
    TO_DWORD(Buffer[4]) OR 
    SHL(TO_DWORD(Buffer[5]), USINT#8) OR 
    SHL(TO_DWORD(Buffer[6]), USINT#16) OR 
    SHL(TO_DWORD(Buffer[7]), USINT#24)
);

                             
            TotalPacketSize := 8 + TO_INT(PayloadLength);

            // 4. CHECK COMPLETENESS: Ensure the complete payload length has arrived
            IF BufferLength < TotalPacketSize THEN
                EXIT; // Packet incomplete. Stop parsing and wait for the rest of the stream
            END_IF;

            // 5. EXTRACT PAYLOAD: Complete packet found! Copy it to output array
            FOR i := 0 TO TO_INT(PayloadLength) - 1 DO
                ExtractedPayload[i] := Buffer[8 + i];
            END_FOR;
            
            ExtractedLength := TO_INT(PayloadLength);
            PayloadReady := True; // Signal to consumer logic that data is ready
            
            // Execute application-specific logic with ExtractedPayload here if needed...

            // 6. SLIDE STREAM BUFFER: Shift remaining trailing data to index 0
            BufferLength := BufferLength - TotalPacketSize;
            IF BufferLength > 0 THEN
                FOR i := 0 TO (BufferLength - 1) DO
                    Buffer[i] := Buffer[TotalPacketSize + i];
                END_FOR;
            END_IF;

        END_WHILE;
    END_METHOD
END_CLASS

CLASS TCP_StreamParser 

    VAR CONSTANT
        BUFFER_LENGTH : INT := 4095;
        PAYLOAD_LENGTH : INT := 2048;
        HEADER_SIZE : INT := 8;
    END_VAR

    VAR
        // Persistent internal stream buffer
        Buffer : Array[0..BUFFER_LENGTH] Of Byte;
        BufferLength : Int := 0; // Tracks current number of valid bytes in Buffer
    END_VAR

    VAR PUBLIC
        // Parsing state variables
        MagicFound : Bool := False;
        MarkerIndex : Int := 0;
        PayloadLength : DInt := 0;
        TotalPacketSize : Int := 0;
        
        // Output / Export data interface
        PayloadReady : Bool := False;
        PayloadReadyCount : Int := 0;

        ExtractedPayload : Array[0..PAYLOAD_LENGTH] Of Byte;
        ExtractedLength : Int := 0;
    END_VAR

    METHOD PUBLIC Execute
        VAR_IN_OUT
            // Dynamic bounds array to safely read variable network chunk sizes
            RxTemp : Array[*] Of Byte;
            RxCount : Int;
        END_VAR
        
        VAR_TEMP
            i : INT;
            UpperIndexBound : DINT;
        END_VAR
        
        PayloadReady := False; // Reset flag every cycle

        // 1. APPEND NEW NETWORK DATA TO STREAM BUFFER
        IF RxCount > 0 THEN
            // FIX 1: Dynamically match against the upper bound of the constant size (4095 index = 4096 size)
            IF (BufferLength + RxCount) <= (BUFFER_LENGTH + 1) THEN
                FOR i := 0 TO RxCount - 1 DO
                    Buffer[BufferLength + i] := RxTemp[LOWER_BOUND(RxTemp, 1) + TO_DINT(i)];
                END_FOR;
                BufferLength := BufferLength + RxCount;
            ELSE
                // Buffer Overflow Safety: Wipe pointer to avoid crashing the PLC PLC
                BufferLength := 0;
            END_IF;
        END_IF;

        // 2. PARSING LOOP: Process internal stream buffer
        WHILE BufferLength >= HEADER_SIZE DO 
            
            MagicFound := False;
            
            // Look for the 4-byte Magic Marker: 0x12345678
            FOR i := 0 TO (BufferLength - 4) DO
                IF Buffer[i] = TO_BYTE(16#12) AND Buffer[i+1] = TO_BYTE(16#34) AND Buffer[i+2] = TO_BYTE(16#56) AND Buffer[i+3] = TO_BYTE(16#78) THEN
                    MagicFound := True;
                    MarkerIndex := i;
                    EXIT; // Found the earliest marker, break FOR loop
                END_IF;
            END_FOR;

            // FIX 2: Handle garbage data or unmatched fragments safely without losing potential split markers
            IF NOT MagicFound THEN
                IF BufferLength > 3 THEN
                    FOR i := 0 TO 2 DO
                        Buffer[i] := Buffer[BufferLength - 3 + i];
                    END_FOR;
                    BufferLength := 3;
                END_IF;
                EXIT; // Force break of WHILE loop to wait for more data
            END_IF;

            // If the magic marker was found deeper in the stream, slide buffer to align to it
            IF MarkerIndex > 0 THEN
                FOR i := 0 TO (BufferLength - 1 - MarkerIndex) DO
                    Buffer[i] := Buffer[MarkerIndex + i];
                END_FOR;
                BufferLength := BufferLength - MarkerIndex;
            END_IF;

            // Re-verify we still have a header window left after shifting
            IF BufferLength < HEADER_SIZE THEN
                EXIT; 
            END_IF;

            // 3. EXTRACT LENGTH (little-endian UDINT from fixed magic frame)
            PayloadLength := TO_DINT(
                TO_DWORD(Buffer[4]) OR 
                SHL(TO_DWORD(Buffer[5]), USINT#8) OR 
                SHL(TO_DWORD(Buffer[6]), USINT#16) OR 
                SHL(TO_DWORD(Buffer[7]), USINT#24)
            );

            TotalPacketSize := HEADER_SIZE + TO_INT(PayloadLength);

            // FIX 3: Safety verification against internal memory sizing bounds
            IF TotalPacketSize > (BUFFER_LENGTH + 1) OR TO_INT(PayloadLength) > (PAYLOAD_LENGTH + 1) THEN
                // Protocol violation or data corruption occurred. Flush buffer to reset syncing alignment.
                BufferLength := 0;
                EXIT;
            END_IF;

            // 4. CHECK COMPLETENESS: Ensure full payload has arrived
            IF BufferLength < TotalPacketSize THEN
                EXIT; // Stop parsing this cycle, await more TCP fragments
            END_IF;

            // 5. EXTRACT PAYLOAD
            FOR i := 0 TO TO_INT(PayloadLength) - 1 DO
                ExtractedPayload[i] := Buffer[HEADER_SIZE + i];
            END_FOR;
            
            ExtractedLength := TO_INT(PayloadLength);
            PayloadReady := True; 

            IF PayloadReady THEN
                PayloadReadyCount := PayloadReadyCount + 1;
            END_IF;
                

            // FIX 4: Put application callback trigger HERE if handling multiple frames in 1 cycle
            // This ensures every parsed message gets intercepted before being overwritten below

            // 6. SLIDE STREAM BUFFER AHEAD
            BufferLength := BufferLength - TotalPacketSize;
            IF BufferLength > 0 THEN
                FOR i := 0 TO (BufferLength - 1) DO
                    Buffer[i] := Buffer[TotalPacketSize + i];
                END_FOR;
            END_IF;

            // FIX 5: Prevent infinite loops if a corrupted packet sets PayloadLength to 0
            IF TotalPacketSize <= HEADER_SIZE THEN
                BufferLength := 0;
                EXIT;
            END_IF;

        END_WHILE;
    END_METHOD
END_CLASS
