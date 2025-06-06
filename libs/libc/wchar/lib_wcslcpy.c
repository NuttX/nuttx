/****************************************************************************
 * libs/libc/wchar/lib_wcslcpy.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 1998 Todd C. Miller <Todd.Miller@courtesan.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <wchar.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wcslen
 *
 * Description:
 *   Copy src to string dst of size "size".  At most size-1 characters
 *   will be copied.  Always NUL terminates (unless size == 0).
 *   Returns wcslen(src); if retval >= size, truncation occurred.
 *
 ****************************************************************************/

size_t wcslcpy(FAR wchar_t *dst, FAR const wchar_t *src, size_t size)
{
  FAR wchar_t *d = dst;
  FAR const wchar_t *s = src;
  size_t n = size;

  /* Copy as many bytes as will fit */

  if (n != 0 && --n != 0)
    {
      do
        {
          if ((*d++ = *s++) == 0)
            {
              break;
            }
        }
      while (--n != 0);
    }

  /* Not enough room in dst, add NUL and traverse rest of src */

  if (n == 0)
    {
      if (size != 0)
        {
          *d = '\0';            /* NUL-terminate dst */
        }
      while (*s++);
    }

  return (s - src - 1);         /* count does not include NUL */
}
