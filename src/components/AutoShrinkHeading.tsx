import { useLayoutEffect, useRef } from 'react'

type AutoShrinkHeadingProps = {
  text: string
}

const AUTO_HEADING_MAX_FONT_SIZE_PX = 28
const AUTO_HEADING_MIN_FONT_SIZE_PX = 11

function clamp(value: number, minValue: number, maxValue: number) {
  return Math.min(maxValue, Math.max(minValue, value))
}

export function AutoShrinkHeading({ text }: AutoShrinkHeadingProps) {
  const headingRef = useRef<HTMLHeadingElement | null>(null)
  const textRef = useRef<HTMLSpanElement | null>(null)

  useLayoutEffect(() => {
    const heading = headingRef.current
    const label = textRef.current
    if (!heading || !label) {
      return
    }

    let frame = 0

    const updateFontSize = () => {
      frame = 0

      const availableWidth = heading.clientWidth
      if (availableWidth <= 0) {
        return
      }

      label.style.fontSize = `${AUTO_HEADING_MAX_FONT_SIZE_PX}px`
      const naturalWidth = label.scrollWidth
      if (naturalWidth <= 0) {
        return
      }

      const fittedSize = clamp(
        (AUTO_HEADING_MAX_FONT_SIZE_PX * availableWidth) / naturalWidth,
        AUTO_HEADING_MIN_FONT_SIZE_PX,
        AUTO_HEADING_MAX_FONT_SIZE_PX,
      )
      label.style.fontSize = `${fittedSize}px`
    }

    const scheduleUpdate = () => {
      if (frame !== 0) {
        return
      }
      frame = window.requestAnimationFrame(updateFontSize)
    }

    updateFontSize()

    const resizeObserver = new ResizeObserver(scheduleUpdate)
    resizeObserver.observe(heading)

    return () => {
      resizeObserver.disconnect()
      if (frame !== 0) {
        window.cancelAnimationFrame(frame)
      }
    }
  }, [text])

  return (
    <h2 ref={headingRef} className="auto-shrink-heading">
      <span ref={textRef} className="auto-shrink-heading__text">
        {text}
      </span>
    </h2>
  )
}
