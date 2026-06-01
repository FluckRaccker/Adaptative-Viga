LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE ieee.numeric_std.ALL;
USE ieee.std_logic_signed.ALL;
--USE WORK.OPER_32B.ALL;

PACKAGE BASIC_OP IS
	
	CONSTANT MIN_16 : STD_LOGIC_VECTOR (15 DOWNTO 0) := X"8000";
	CONSTANT MAX_16 : STD_LOGIC_VECTOR (15 DOWNTO 0) := X"7FFF";
	CONSTANT MIN_32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := X"80000000";
	CONSTANT MAX_32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := X"7FFFFFFF";
	
	FUNCTION ABS_S(ABS_ARG: STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_MAC(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_ADD(L_var1, L_var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_SUB(L_var1, L_var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION EXTRACT_H(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION EXTRACT_L(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION EXTRACT_LS(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION ROUND(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_DEPOSIT_H(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_DEPOSIT_L(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION SATURATE(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION SUB(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	
	FUNCTION ADD(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION MULT_R(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION I_MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_MSU(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION MSU_R(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION NEGATE(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_NEGATE(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION MAC_R(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_ABS(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION NORM_S(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION NORM_L(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	
--	FUNCTION S_SHL_MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION S_SHL(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION S_SHR(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_SHL(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_SHR(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_SHLNS(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION S_SHR_R(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_SHR_R(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION S_MIN(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION S_MAX(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION DIV_S(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION L_COMP(hi, lo : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION MPY_32(hi1, lo1, hi2, lo2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	FUNCTION MPY_32_16(hi, lo, n : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR;
	
END PACKAGE BASIC_OP;

PACKAGE BODY BASIC_OP IS
	
	FUNCTION ABS_S(ABS_ARG: STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 37 TLE
		VARIABLE ABS_VAL : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF ABS_ARG(15) = '0' THEN
			ABS_VAL := 	ABS_ARG;
		ELSIF ABS_ARG = MIN_16 THEN
			ABS_VAL := MAX_16;
		ELSE
			ABS_VAL := -ABS_ARG;
		END IF;
		
		RETURN ABS_VAL;
		
	END FUNCTION ABS_S;

	------------------------------

	FUNCTION L_MAC(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 107 TLE, 2 MULT
		VARIABLE acum : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		acum := L_ADD(L_var3, L_MULT(var1, var2));
		RETURN acum;
	END FUNCTION L_MAC;
	
	------------------------------

	FUNCTION L_MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 43 TLE, 2 MULT
		VARIABLE PROD : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		PROD := var1 * var2;
		IF PROD /= X"40000000" THEN
			PROD := (PROD(30 DOWNTO 0) & '0');
		ELSE
			PROD := MAX_32;
		END IF;
		RETURN PROD;
	END FUNCTION L_MULT;
	
	------------------------------
	
	FUNCTION L_ADD(L_var1, L_var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 64 TLE
		VARIABLE SOMA : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		SOMA := L_var1 + L_var2;
		IF L_var1(31) = L_var2(31) THEN
			IF L_var1(31) /= SOMA(31) THEN
				IF L_var1(31) = '1' THEN
					SOMA := MIN_32;
				ELSE
					SOMA := MAX_32;
				END IF;
			END IF;
		END IF;
		RETURN SOMA;
	END FUNCTION L_ADD;
	
	------------------------------
	
	FUNCTION L_SUB(L_var1, L_var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 64 TLE
		VARIABLE DIF : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		DIF := L_var1 - L_var2;
		IF L_var1(31) /= L_var2(31) THEN
			IF L_var1(31) /= DIF(31) THEN
				IF L_var1(31) = '1' THEN
					DIF := MIN_32;
				ELSE
					DIF := MAX_32;
				END IF;
			END IF;
		END IF;
		RETURN DIF;
	END FUNCTION L_SUB;

	------------------------------

	FUNCTION EXTRACT_H(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 0 TLE
		VARIABLE AUX : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		AUX := L_var1(31 DOWNTO 16);
		RETURN AUX;
	END FUNCTION EXTRACT_H;

	------------------------------

	FUNCTION EXTRACT_L(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 0 TLE
		VARIABLE AUX : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		AUX := L_var1(15 DOWNTO 0);
		RETURN AUX;
	END FUNCTION EXTRACT_L;
	
	------------------------------
	
	FUNCTION EXTRACT_LS(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 0 TLE
		VARIABLE aux32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
		VARIABLE aux16 : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
		VARIABLE L_var_out : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		aux16 := EXTRACT_H(L_var1);
		aux32 := L_SHR(L_var1, X"0001");
		L_var_out := L_MSU(aux32, aux16, X"4000");
		RETURN EXTRACT_L(L_var_out);
	END FUNCTION EXTRACT_LS;
	
	------------------------------
	
	FUNCTION ROUND(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 32 TLE
		VARIABLE AUX : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		AUX := L_ADD(L_var1, X"00008000");
		RETURN EXTRACT_H(AUX);
	END FUNCTION ROUND;

	------------------------------
	
	FUNCTION L_DEPOSIT_H(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 0 TLE
		VARIABLE AUX : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		AUX := (var1 & X"0000");
		RETURN AUX;		
	END FUNCTION L_DEPOSIT_H;

	------------------------------
	
	FUNCTION L_DEPOSIT_L(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 0 TLE
		VARIABLE AUX : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		AUX(31 DOWNTO 16) := (OTHERS => var1(15));
		AUX(15 DOWNTO 0) := var1;
		RETURN AUX;		
	END FUNCTION L_DEPOSIT_L;

	------------------------------

	FUNCTION SATURATE(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 27 TLE
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF L_var1 > X"00007FFF" THEN
			var_out := X"7FFF";
		ELSIF L_var1 < X"FFFF8000" THEN
			var_out := X"8000";
		ELSE
			var_out := EXTRACT_L(L_var1);
		END IF;
		RETURN var_out;
	END FUNCTION SATURATE;

	------------------------------
	
	FUNCTION SUB(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 32 TLE
		VARIABLE L_diff : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		L_diff := STD_LOGIC_VECTOR(SIGNED(L_DEPOSIT_L(var1) - L_DEPOSIT_L(var2)));
		var_out := SATURATE(L_diff);
		RETURN var_out;
	END FUNCTION SUB;
	-- Não usar dentro de máquinas de estados, a função SATURATE() não consegue resolver a tempo e o valor volta estourado.

	------------------------------
	
	FUNCTION ADD(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 32 TLE
		VARIABLE L_sum : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		L_sum := L_DEPOSIT_L(var1) + L_DEPOSIT_L(var2);
		RETURN SATURATE(L_sum);
	END FUNCTION ADD;
	-- Não usar dentro de máquinas de estados, a função SATURATE() não consegue resolver a tempo e o valor volta estourado.
	
	------------------------------
	
	FUNCTION MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 15 TLE, 2 MULT
		VARIABLE L_product : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
		VARIABLE aux : STD_LOGIC := '0';
	BEGIN
		L_product := var1 * var2;
		L_product := L_product AND X"FFFF8000";
		aux := L_product(31);
		L_product(16 DOWNTO 0) := L_product(31 DOWNTO 15);
		L_product(31 DOWNTO 17) := (OTHERS => aux);
		IF L_product(16) = '1' THEN
			L_product := L_product OR X"FFFF0000";
		END IF;
		RETURN SATURATE(L_product);
		
	END FUNCTION MULT;
	
	------------------------------
	
	FUNCTION MULT_R(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 46 TLE, 2 MULT
	BEGIN
		RETURN ROUND(L_MULT(var1, var2));
	END FUNCTION MULT_R;
	
	------------------------------
	
	FUNCTION I_MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 26 TLE, 2 MULT
	BEGIN
		RETURN EXTRACT_L(L_SHR(L_MULT(var1, var2), X"0001"));
	END FUNCTION I_MULT;
	
	------------------------------
	
	FUNCTION L_MSU(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS --107 TLE, 2 MULT
		VARIABLE L_product : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
		VARIABLE L_var_out : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		L_product := L_MULT(var1, var2);
		L_var_out := L_SUB(L_var3, L_product);
		RETURN L_var_out;
	END FUNCTION L_MSU;
	
	------------------------------
	
	FUNCTION MSU_R(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 126 TLE, 2 MULT	
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		var_out := ROUND(L_MSU(L_var3, var1, var2));
		RETURN var_out;
	END FUNCTION MSU_R;
	
	------------------------------
	
	FUNCTION NEGATE(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 37 TLE
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var1 /= MIN_16 THEN
			var_out := -var1;
		ELSE
			var_out := MAX_16;
		END IF;
		RETURN var_out;
	END FUNCTION NEGATE;
	
	------------------------------
	
	FUNCTION L_NEGATE(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 74 TLE
		VARIABLE L_var_out : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF L_var1 = MIN_32 THEN
			L_var_out := MAX_32;
		ELSE
			L_var_out := -L_var1;
		END IF;
		RETURN L_var_out;
	END FUNCTION L_NEGATE;
	
	------------------------------
	
	FUNCTION MAC_R(L_var3, var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 126 TLE, 2 MULT
		VARIABLE aux : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		aux := L_MAC(L_var3, var1, var2);
		aux := L_ADD(aux, X"00008000");
		RETURN EXTRACT_H(aux);
	END FUNCTION MAC_R;
	
	------------------------------
	
	FUNCTION L_ABS(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 75 TLE
		VARIABLE L_var_out : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF L_var1 = MIN_32 THEN
			L_var_out := MAX_32;
		ELSIF L_var1(31) = '1' THEN
			L_var_out := -L_var1;
		ELSE
			L_var_out := L_var1;
		END IF;
		RETURN L_var_out;
	END FUNCTION L_ABS;
	
	------------------------------
	
	FUNCTION NORM_S(var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 35 TLE
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var1 = X"0000" THEN
			var_out := X"000F";
		ELSIF var1 = X"FFFF" THEN
			var_out := X"000F";
		ELSE
			IF var1(14) /= var1(15) THEN
			--IF (var1 AND X"8000") /= (var1 AND X"4000") THEN
				var_out := X"0000";
			ELSIF var1(13) /= var1(15) THEN
				var_out := X"0001";
			ELSIF var1(12) /= var1(15) THEN
				var_out := X"0002";
			ELSIF var1(11) /= var1(15) THEN
				var_out := X"0003";
			ELSIF var1(10) /= var1(15) THEN
				var_out := X"0004";
			ELSIF var1(9) /= var1(15) THEN
				var_out := X"0005";
			ELSIF var1(8) /= var1(15) THEN
				var_out := X"0006";
			ELSIF var1(7) /= var1(15) THEN
				var_out := X"0007";
			ELSIF var1(6) /= var1(15) THEN
				var_out := X"0008";
			ELSIF var1(5) /= var1(15) THEN
				var_out := X"0009";
			ELSIF var1(4) /= var1(15) THEN
				var_out := X"000A";
			ELSIF var1(3) /= var1(15) THEN
				var_out := X"000B";
			ELSIF var1(2) /= var1(15) THEN
				var_out := X"000C";
			ELSIF var1(1) /= var1(15) THEN
				var_out := X"000D";
			ELSE
				var_out := X"000E";
			END IF;
		END IF;
		
		RETURN var_out;
	END FUNCTION NORM_S;
	
	------------------------------
	
	FUNCTION NORM_L(L_var1 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 83 TLE
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF L_var1 = X"00000000" THEN
			--var_out := X"0000";
			var_out := X"001F";
		ELSIF L_var1 = X"FFFFFFFF" THEN
			var_out := X"001F";
		ELSE
			IF L_var1(30) /= L_var1(31) THEN
				var_out := X"0000";
			ELSIF L_var1(29) /= L_var1(31) THEN
				var_out := X"0001";
			ELSIF L_var1(28) /= L_var1(31) THEN
				var_out := X"0002";
			ELSIF L_var1(27) /= L_var1(31) THEN
				var_out := X"0003";
			ELSIF L_var1(26) /= L_var1(31) THEN
				var_out := X"0004";
			ELSIF L_var1(25) /= L_var1(31) THEN
				var_out := X"0005";
			ELSIF L_var1(24) /= L_var1(31) THEN
				var_out := X"0006";
			ELSIF L_var1(23) /= L_var1(31) THEN
				var_out := X"0007";
			ELSIF L_var1(22) /= L_var1(31) THEN
				var_out := X"0008";
			ELSIF L_var1(21) /= L_var1(31) THEN
				var_out := X"0009";
			ELSIF L_var1(20) /= L_var1(31) THEN
				var_out := X"000A";
			ELSIF L_var1(19) /= L_var1(31) THEN
				var_out := X"000B";
			ELSIF L_var1(18) /= L_var1(31) THEN
				var_out := X"000C";
			ELSIF L_var1(17) /= L_var1(31) THEN
				var_out := X"000D";
			ELSIF L_var1(16) /= L_var1(31) THEN
				var_out := X"000E";
			ELSIF L_var1(15) /= L_var1(31) THEN
				var_out := X"000F";
			ELSIF L_var1(14) /= L_var1(31) THEN
				var_out := X"0010";
			ELSIF L_var1(13) /= L_var1(31) THEN
				var_out := X"0011";
			ELSIF L_var1(12) /= L_var1(31) THEN
				var_out := X"0012";
			ELSIF L_var1(11) /= L_var1(31) THEN
				var_out := X"0013";
			ELSIF L_var1(10) /= L_var1(31) THEN
				var_out := X"0014";
			ELSIF L_var1(9) /= L_var1(31) THEN
				var_out := X"0015";
			ELSIF L_var1(8) /= L_var1(31) THEN
				var_out := X"0016";
			ELSIF L_var1(7) /= L_var1(31) THEN
				var_out := X"0017";
			ELSIF L_var1(6) /= L_var1(31) THEN
				var_out := X"0018";
			ELSIF L_var1(5) /= L_var1(31) THEN
				var_out := X"0019";
			ELSIF L_var1(4) /= L_var1(31) THEN
				var_out := X"001A";
			ELSIF L_var1(3) /= L_var1(31) THEN
				var_out := X"001B";
			ELSIF L_var1(2) /= L_var1(31) THEN
				var_out := X"001C";
			ELSIF L_var1(1) /= L_var1(31) THEN
				var_out := X"001D";
			ELSE
				var_out := X"001E";
			END IF;
		END IF;
		
		RETURN var_out;
	END FUNCTION NORM_L;
	
	------------------------------
	
--	FUNCTION S_SHL_MULT(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS
--		VARIABLE num : INTEGER RANGE 0 TO 32768 := 0;
--		VARIABLE comp : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
--		VARIABLE aux : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
--		VARIABLE fator : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
--		VARIABLE Mult32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
--		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
--	BEGIN
--		IF var2(15) = '0' THEN
--			aux := NORM_S(var1);
--			IF var2 > aux THEN
--				IF var1(15) = '0' THEN
--					var_out := MAX_16;
--				ELSE
--					var_out := MIN_16;
--				END IF;
--			ELSE
--				num := TO_INTEGER(SIGNED(var2));
--				num := 2 ** num;
--				fator := STD_LOGIC_VECTOR(TO_SIGNED(num, 16));
--				Mult32 := var1 * fator;
--				var_out := Mult32 (15 DOWNTO 0);
--			END IF;
--		ELSE
--			comp := -var2;
--			IF var2 < X"0010" THEN
--				comp := X"0010";
--			END IF;
--			var_out := (OTHERS => '0');
--		END IF;
--		RETURN var_out;
--	END FUNCTION S_SHL_MULT;

	------------------------------
	
	FUNCTION S_SHL(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 125 TLE
		VARIABLE aux : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		aux := NORM_S(var1);
		IF var2 > aux THEN
			IF var1(15) = '0' THEN
				var_out := MAX_16;
			ELSE
				var_out := MIN_16;
			END IF;
		ELSE
			var_out := SHL(var1, var2); -- Função de Shift localizada na Biblioteca std_logic_signed.
		END IF;
		RETURN var_out;
	END FUNCTION S_SHL;
	
	------------------------------
	
	FUNCTION S_SHR(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 74 TLE
		VARIABLE var_out : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var2 >= X"000F" THEN
			IF var1(15) = '0' THEN
				var_out := (OTHERS => '0');
			ELSE
				var_out := (OTHERS => '1');
			END IF;
		ELSE
			var_out := NOT(var1);
			var_out := SHR(var_out, var2);
			var_out := NOT(var_out);
		END IF;
		RETURN var_out;
	END FUNCTION S_SHR;
	
	------------------------------
	
	FUNCTION L_SHL(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 271 TLE
		VARIABLE aux : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
		VARIABLE L_var_out : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		aux := NORM_L(L_var1);
		IF var2 > aux THEN
			IF L_var1(31) = '0' THEN
				L_var_out := MAX_32;
			ELSE
				L_var_out := MIN_32;
			END IF;
		ELSE
			L_var_out := SHL(L_var1, var2); -- Função de Shift localizada na Biblioteca std_logic_signed.
		END IF;
		RETURN L_var_out;
	END FUNCTION L_SHL;
	
	------------------------------
	
	FUNCTION L_SHR(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 170 TLE
		VARIABLE L_var_out : STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var2 >= X"001F" THEN
			IF L_var1(31) = '0' THEN
				L_var_out := (OTHERS => '0');
			ELSE
				L_var_out := (OTHERS => '1');
			END IF;
		ELSE
			L_var_out := NOT(SHR(NOT(L_var1), var2));
		END IF;
		RETURN L_var_out;
	END FUNCTION L_SHR;
	
	------------------------------
	
	FUNCTION L_SHLNS(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 169 TLE
	BEGIN
		RETURN SHL(L_var1, var2); -- Função de Shift localizada na Biblioteca std_logic_signed.
	END FUNCTION L_SHLNS;
	
	------------------------------
	
	FUNCTION S_SHR_R(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 122 TLE
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var2 > X"000F" THEN
			var_out := (OTHERS => '0');
		ELSE
			var_out := S_SHR(var1, var2);
			IF var1(TO_INTEGER(SIGNED(var2)) - 1) = '1' THEN
				var_out := var_out + X"0001";
			END IF;
		END IF;
		RETURN var_out;
	END FUNCTION S_SHR_R;
	
	------------------------------
	
	FUNCTION L_SHR_R(L_var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 255 TLE
		VARIABLE L_var_out : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var2 > X"001F" THEN
			L_var_out := (OTHERS => '0');
		ELSE
			L_var_out := L_SHR(L_var1, var2);
			IF L_var1(TO_INTEGER(SIGNED(var2)) - 1) = '1' THEN
				L_var_out := L_var_out + X"00000001";
			END IF;
		END IF;
		RETURN L_var_out;
	END FUNCTION L_SHR_R;
	
	------------------------------
	
	FUNCTION S_MIN(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS
		VARIABLE var_out : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var1 < var2 THEN
			var_out := var1;
		ELSE
			var_out := var2;
		END IF;
		RETURN var_out;
	END FUNCTION S_MIN;
	
	------------------------------
	
	FUNCTION S_MAX(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS
		VARIABLE var_out : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var1 > var2 THEN
			var_out := var1;
		ELSE
			var_out := var2;
		END IF;
		RETURN var_out;
	END FUNCTION S_MAX;
	
	------------------------------
	
--	FUNCTION DIV_S(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS
--		VARIABLE it : NATURAL RANGE 0 TO 15 := 0;
--		VARIABLE L_num : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
--		VARIABLE L_denom : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
--		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
--	BEGIN
--		IF var1 > var2 OR var1(15) = '1' OR var2(15) = '1' THEN
--			var_out := (OTHERS => '0'); -- Erro na divisão
--		ELSIF var2 = X"0000" THEN
--			var_out := (OTHERS => '0'); -- Divisão por zero
--		ELSE
--			IF var1 = X"0000" THEN
--				var_out := (OTHERS => '0');
--			ELSIF var1 = var2 THEN
--				var_out := MAX_16;
--			ELSE
--				L_num := L_DEPOSIT_L(var1);
--				L_denom := L_DEPOSIT_L(var2);
--				WHILE it < 15 LOOP
--					var_out(15 DOWNTO 0) := (var_out(14 DOWNTO 0) & '0');
--					L_num(31 DOWNTO 0) := (L_num(30 DOWNTO 0) & '0');
--					IF L_num >= L_denom THEN
--						L_num := L_SUB(L_num, L_denom);
--						var_out := ADD(var_out, X"0001");					
--					END IF;
--					it := it + 1;
--				END LOOP;
--			END IF;
--		END IF;
--		RETURN var_out;
--	END FUNCTION DIV_S;
	
	FUNCTION DIV_S(var1, var2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 888 TLE
		VARIABLE it : NATURAL RANGE 0 TO 15 := 0;
		VARIABLE L_num : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
		VARIABLE L_denom : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
		VARIABLE var_out : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
		VARIABLE aux32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
		VARIABLE aux16 : STD_LOGIC_VECTOR (15 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		IF var1 > var2 OR var1(15) = '1' OR var2(15) = '1' THEN
			var_out := (OTHERS => '0'); -- Erro na divisão
		ELSIF var2 = X"0000" THEN
			var_out := (OTHERS => '0'); -- Divisão por zero
		ELSE
			IF var1 = X"0000" THEN
				var_out := (OTHERS => '0');
			ELSIF var1 = var2 THEN
				var_out := MAX_16;
			ELSE
				aux32 := (var1(15) & var1 & X"000" & O"0");
				aux32 := STD_LOGIC_VECTOR(UNSIGNED(aux32) / UNSIGNED(var2));
				var_out := aux32(15 DOWNTO 0);
			END IF;
		END IF;
		RETURN var_out;
	END FUNCTION DIV_S;

	------------------------------
	
	FUNCTION L_COMP(hi, lo : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 48 TLE
		VARIABLE L_32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		L_32 := L_DEPOSIT_H(hi);
		L_32 := L_MAC(L_32, lo, X"0001");
		RETURN L_32;
	END FUNCTION L_COMP;
	
	------------------------------
	
	FUNCTION MPY_32(hi1, lo1, hi2, lo2 : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 198 TLE, 6 MULT
		VARIABLE L_32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		L_32 := L_MULT(hi1, hi2);
		L_32 := L_MAC(L_32, MULT(hi1, lo2), X"0001");
		L_32 := L_MAC(L_32, MULT(lo1, hi2), X"0001");
		RETURN L_32;
	END FUNCTION MPY_32;
	
	------------------------------
	
	FUNCTION MPY_32_16(hi, lo, n : STD_LOGIC_VECTOR) RETURN STD_LOGIC_VECTOR IS -- 43 TLE, 2 MULT
		VARIABLE L_32 : STD_LOGIC_VECTOR (31 DOWNTO 0) := (OTHERS => '0');
	BEGIN
		L_32 := L_MULT(hi, n);
		L_32 := L_MAC(L_32, MULT(lo, n), X"0000");
		RETURN L_32;
	END FUNCTION MPY_32_16;
	
	------------------------------


END PACKAGE BODY;